"""Named workstation backend factories; simulator backends register here later."""

from gamepad_vla.workstation.backends import NullBackend, RealRos2Backend


def _null_factory(spec, **options):
    return NullBackend()


def _real_factory(spec, **options):
    return RealRos2Backend(
        options.get("node_factory"), enable_real=options["enable_real"],
        arm_units=options["arm_units"], sides=options["real_sides"])


def _sapien_factory(spec, **options):
    from gamepad_vla.workstation.sapien_backend import SapienBackend  # needs SAPIEN 3.0.3

    return SapienBackend(spec, work_dir=options.get("work_dir"))


BACKENDS = {
    "null": _null_factory,
    "real": _real_factory,
    "sapien": _sapien_factory,
}
BACKEND_FACTORIES = BACKENDS
