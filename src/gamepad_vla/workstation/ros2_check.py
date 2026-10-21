"""Real-rclpy isolated transport regression and optional SAPIEN acceptance.

No hardware inputs, SDKs, or renderer. Publishers are restricted to a test prefix
and a non-default domain; collection uses the production source and recorder.
"""
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np

from .collection_sources import Ros2Source, ros_command_topics
from .arm_mapping import ArmMapper
from .episode_schema import EpisodeWriter, Observation, read_episode, read_input_events


def check_ros2(output, *, duration=2., replay=None, backend_name=None, control_hz=60,
               topic_prefix="/gamepad_vla_test/regression/"):
    if os.environ.get("ROS_DOMAIN_ID", "0") == "0" or not topic_prefix.startswith("/gamepad_vla_test/"):
        raise RuntimeError("real ROS test requires explicit non-default ROS_DOMAIN_ID and /gamepad_vla_test/ topics")
    import rclpy
    from sensor_msgs.msg import JointState
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    topics = ros_command_topics(topic_prefix)
    spec = backend = session = source = publisher = thread = None
    writer = None
    rclpy.init(args=[])
    published, applications, publisher_errors = [], [], []
    finished = threading.Event()
    try:
        from gamepad_vla.site import configure_runtime
        configure_runtime()
        if backend_name:
            from .collection import create_backend, Session
            from .spec import load_spec
            spec = load_spec(Path(__file__).resolve().parents[3] / "configs/workcell/station_g20_v4_4.json")
            backend = create_backend(backend_name, spec, work_dir=output / "physics", render=False)
            backend.reset()
        startup = {s: getattr(spec, s + "_arm").home_q if spec else [0.]*7 for s in ("left", "right")}
        mapper = ArmMapper(startup)
        if backend:
            session = Session(output / "recording", spec, backend,
                              dict(kind="ros2", topics=topics, test=True), mapper=mapper, fps=control_hz)
            session.control(dict(action="start"))
        else:
            writer = EpisodeWriter(output / "recording/episode-000000",
                                   dict(scene_spec_text="{}", fps=control_hz, input_source="ROS transport regression; synthetic observations"))
        source = Ros2Source(mapper, topic_prefix=topic_prefix)
        publisher = rclpy.create_node("gamepad_vla_test_publisher", enable_rosout=False, start_parameter_services=False)
        pubs = {k: publisher.create_publisher(JointState, v, 128) for k, v in topics.items()}
        deadline = time.monotonic()+15
        while not all(p.get_subscription_count() == 1 for p in pubs.values()):
            if time.monotonic()>deadline:
                raise RuntimeError("test subscriptions not discovered")
            time.sleep(.01)
        count = math.ceil(duration*100)
        actions = times = None
        if replay:
            with np.load(replay, allow_pickle=False) as z:
                actions=z["action"].copy(); times=z["timestamp"].astype(float); times-=times[0]
            cycle=float(times[-1]+np.median(np.diff(times)))
        def send():
            try:
                start=time.monotonic()
                with (output/"published.jsonl").open("x",buffering=1) as stream:
                    for i in range(count):
                        time.sleep(max(0,start+i/100-time.monotonic()))
                        ix=max(0,np.searchsorted(times,(i/100)%cycle,side="right")-1) if replay else None
                        for key,pub in pubs.items():
                            side,kind=key.split("_")
                            if replay:
                                offset=(14+(0 if side=="left" else 20)) if kind=="hand" else (0 if side=="left" else 7)
                                values=actions[ix,offset:offset+(20 if kind=="hand" else 7)].astype(float)
                                if kind=="arm": values=np.rad2deg(values)  # synthetic degree-valued transport input, not recovered master trajectory
                                values=values.tolist()
                            else:
                                values=[127.125+i*.001]*20 if kind=="hand" else [i*.001]*7
                                values[-1]=-0.0  # assert binary64 preservation, including signed zero
                            msg=JointState(); ns=time.time_ns()
                            msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9
                            msg.header.frame_id=str(i);msg.position=values
                            msg.name=[f"joint_{j}" for j in range(len(values))]
                            entry=dict(channel=key,index=i,header=dict(stamp=dict(sec=ns//10**9,nanosec=ns%10**9),frame_id=str(i)),
                                       values=values,published_monotonic_ns=time.monotonic_ns(),published_wall_time_ns=time.time_ns())
                            pub.publish(msg);published.append(entry);stream.write(json.dumps(entry)+"\n")
            except Exception as exc:
                publisher_errors.append(exc)
            finally:
                finished.set()
        thread=threading.Thread(target=send,name="test-jointstate-publisher");thread.start()
        begin=time.monotonic(); ticks=0;last_sequences={};seen=set()
        while time.monotonic()-begin < duration+1.:
            time.sleep(max(0,begin+ticks/control_hz-time.monotonic()))
            events=source.drain_events()
            if session: session.record_inputs(events)
            else: writer.append_inputs(events)
            commands=source.poll(time.monotonic())
            if backend:backend.apply(commands)
            applied=time.monotonic_ns()
            for key,sequence in source.polled_sequences.items():
                if sequence not in seen:
                    seen.add(sequence);applications.append(dict(sequence=sequence,channel=key,applied_monotonic_ns=applied))
            last_sequences.update(source.polled_sequences)
            if backend:
                backend.step(1/control_hz)
                session.record(backend.observe(),commands,applied_monotonic_ns=applied,input_sequences=source.polled_sequences)
            else:
                pose=np.array([0.,0.,0.,1.,0.,0.,0.])
                obs=Observation((ticks+1)/control_hz,time.time_ns(),np.zeros((2,21)),np.zeros((2,7)),
                                np.tile(pose,(2,1)),np.tile(pose,(2,5,1)),np.zeros(32),np.ones(32,bool),{},False,"transport-test")
                writer.append(obs,{k:asdict(c) for k,c in zip(source.polled_sequences,commands)}, {},
                              applied_monotonic_ns=applied,input_sequences=last_sequences)
            ticks+=1
        thread.join(timeout=5.)
        if thread.is_alive():raise RuntimeError("test publisher did not finish")
        if publisher_errors:raise RuntimeError("publisher failed") from publisher_errors[0]
        source.close()
        events=source.drain_events()
        if session:session.record_inputs(events);session.close();session=None
        else:writer.append_inputs(events);writer.close();writer=None
        decode_errors=source.decode_errors;source=None
        episode=output/"recording/episode-000000"
        meta,rows=read_episode(episode);events=read_input_events(episode)
        (output/"applications.jsonl").write_text("".join(json.dumps(a)+"\n" for a in applications))
        expected={(e["channel"],e["index"]):e for e in published}
        event_map={e["sequence"]:e for e in events}
        per_channel={};all_receive=[];all_apply=[]
        for key in topics:
            tx=[e for e in published if e["channel"]==key];rx=[e for e in events if e["channel"]==key]
            ids=[int(e["header"]["frame_id"]) for e in rx]
            exact=(ids==list(range(count))) and all(np.asarray(e["raw"]["position"],np.float64).tobytes()==np.asarray(tx[i]["values"],np.float64).tobytes() and e["header"]==tx[i]["header"] for i,e in enumerate(rx))
            receive=[(e["received_monotonic_ns"]-expected[(key,int(e["header"]["frame_id"]))]["published_monotonic_ns"])/1e6 for e in rx]
            apply=[(a["applied_monotonic_ns"]-expected[(key,int(event_map[a["sequence"]]["header"]["frame_id"]))]["published_monotonic_ns"])/1e6 for a in applications if a["channel"]==key]
            all_receive.extend(receive);all_apply.extend(apply)
            monotonic=all(b["received_wall_time_ns"]>=a["received_wall_time_ns"] and (b["header"]["stamp"]["sec"],b["header"]["stamp"]["nanosec"])>(a["header"]["stamp"]["sec"],a["header"]["stamp"]["nanosec"]) for a,b in zip(rx,rx[1:]))
            hz=(count-1)*1e9/(tx[-1]["published_monotonic_ns"]-tx[0]["published_monotonic_ns"])
            per_channel[key]=dict(published=len(tx),recorded=len(rx),bitwise_exact_and_ordered=exact,timestamps_monotonic=monotonic,publish_hz=hz,applied_unique=len(apply),apply_p50_ms=float(np.percentile(apply,50)),apply_p95_ms=float(np.percentile(apply,95)))
        result=dict(channels=per_channel,published_total=len(published),recorded_total=len(events),decode_errors=decode_errors,
                    control_hz=control_hz,rows=len(rows),sim_duration_s=rows[-1]["sim_time_s"],wall_duration_s=time.monotonic()-begin,
                    receive_p50_ms=float(np.percentile(all_receive,50)),receive_p95_ms=float(np.percentile(all_receive,95)),
                    publish_to_apply_p50_ms=float(np.percentile(all_apply,50)),publish_to_apply_p95_ms=float(np.percentile(all_apply,95)),
                    latency_denominator=len(all_apply),latency_semantics="first applications of latest selected targets; every received message remains in command journal, superseded targets are not applied",
                    schema_version=meta["schema_version"],overlay_equal=all(r["overlay"]["values"]==r["gamepad_signal"] for r in rows) if backend else None)
        result["pass_gate"]=all(c["bitwise_exact_and_ordered"] and c["timestamps_monotonic"] for c in per_channel.values()) and result["publish_to_apply_p95_ms"]<=25 and decode_errors==0
        (output/"summary.json").write_text(json.dumps(result,indent=2)+"\n")
        return result
    finally:
        if thread and thread.is_alive():thread.join(timeout=5.)
        if source:
            source.close()
            events=source.drain_events()
            if session:session.record_inputs(events)
            if writer:writer.append_inputs(events)
        if session:session.close("interrupted")
        if writer:writer.close("interrupted")
        if backend:backend.close()
        if publisher:publisher.destroy_node()
        rclpy.shutdown()
