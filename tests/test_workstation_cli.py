import runpy
from pathlib import Path


def test_cli_parser_exposes_phase_a_choices():
    script = Path(__file__).parents[1] / "scripts/run_workstation.py"
    namespace = runpy.run_path(script, run_name="workstation_cli_test")
    parser = namespace["parser"]()
    args = parser.parse_args([
        "--spec", "spec.json", "--source", "scripted", "--backend", "null", "--duration", "0",
    ])
    assert args.rate_hz == 60.0 and not args.enable_real


def test_udp_allowlist_with_operator_view():
    namespace = runpy.run_path(Path(__file__).parents[1] / "scripts/run_workstation.py")
    args = namespace["parser"]().parse_args([
        "--spec", "spec.json", "--source", "udp", "--duration", "0",
        "--udp-bind-host", "127.0.0.1", "--udp-port", "0",
        "--udp-allow-source", "198.51.100.34, 127.0.0.1",
        "--operator-view", "view.yaml", "--operator-run", "run"])
    source = namespace["_source"](args)
    try:
        assert source.allow_source == {"198.51.100.34", "127.0.0.1"}
    finally:
        source.close()
