"""Show the real ROS applications in a private noVNC desktop over an SSH tunnel."""

import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import secrets
import shutil
import signal
import socket
import string
import subprocess
import tempfile
import time
import urllib.error
import urllib.request

from .world import ROOT

DIRECTORY = ROOT / "artifacts/browser"
SESSION = DIRECTORY / "session.json"


def process_token(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def private_write(path, contents):
    # Create with restrictive permissions before writing credentials or PID data.
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as stream:
        os.fchmod(stream.fileno(), 0o600)
        stream.write(contents)


def running_session():
    try:
        session = json.loads(SESSION.read_text())
        if process_token(session["pid"]) == session["process_token"]:
            return session
    except (OSError, ValueError, KeyError):
        pass
    return None


def show_info(session, reveal_password=True):
    print(f"Browser: {session['url']}", flush=True)
    if reveal_password:
        print(f"VNC password: {Path(session['password_file']).read_text().strip()}", flush=True)
    else:
        print("Show the VNC password with: bash scripts/wsl.sh browser-info", flush=True)
    print(f"SSH tunnel: ssh -N -L {session['port']}:127.0.0.1:{session['port']} YOUR_SSH_HOST", flush=True)
    print("In the browser terminal, enter: Deliver all three parcels", flush=True)
    print("Use the bottom taskbar to open RViz. /quit stops this desktop.", flush=True)


def url_ready(url):
    try:
        with urllib.request.urlopen(url, timeout=1) as response:
            return response.status == 200
    except (OSError, urllib.error.URLError):
        return False


class Desktop:
    def __init__(self, port):
        self.port = port
        self.running = True
        self.children = {}
        self.logs = []
        self.runtime = None
        self.env = os.environ.copy()
        self.arranged = set()
        self.window_seen = {}

    def spawn(self, name, command):
        log = (DIRECTORY / f"{name}.log").open("w")
        self.logs.append(log)
        process = subprocess.Popen(command, cwd=ROOT, env=self.env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        self.children[name] = process
        return process

    def wait_ready(self, check, name, timeout=15):
        deadline = time.monotonic()+timeout
        while self.running and time.monotonic() < deadline:
            process = self.children.get(name)
            if process and process.poll() is not None:
                raise RuntimeError(f"{name} exited; see {DIRECTORY / (name+'.log')}")
            if check():
                return
            time.sleep(0.2)
        raise RuntimeError(f"{name} did not become ready; see {DIRECTORY}")

    def start(self):
        required = ("Xtigervnc", "tigervncpasswd", "websockify", "openbox", "tint2",
                    "xterm", "wmctrl", "xdotool", "xauth", "xdpyinfo", "ollama")
        missing = [name for name in required if not shutil.which(name)]
        if missing or not Path("/usr/share/novnc/vnc.html").is_file():
            raise RuntimeError(f"Missing browser dependencies: {missing}. Run scripts/install_browser.sh --apply in Ubuntu-24.04.")
        # Respect the existing simulator/agent locks before allocating a desktop.
        domain = os.environ.get("ROS_DOMAIN_ID", "42")
        for name in ("sim", "agent"):
            with open(f"/tmp/warehouse-{name}-{os.getuid()}-{domain}.lock", "a") as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    raise RuntimeError(f"A warehouse {name} is already running; stop it before starting the browser demo.") from None
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(("127.0.0.1", self.port))
        display = next((n for n in range(42, 62) if not Path(f"/tmp/.X{n}-lock").exists()
                        and not Path(f"/tmp/.X11-unix/X{n}").exists()), None)
        if display is None:
            raise RuntimeError("No free virtual X display in the range :42–:61")
        self.runtime = Path(tempfile.mkdtemp(prefix=f"warehouse-browser-{os.getuid()}-"))
        password_file = DIRECTORY / "password.txt"
        if not password_file.exists():
            password = "".join(secrets.choice(string.ascii_letters+string.digits) for _ in range(8))
            private_write(password_file, password+"\n")
        password_file.chmod(0o600)
        password = password_file.read_text().strip()
        if len(password) != 8 or not password.isascii():
            raise RuntimeError(f"Expected an eight-character VNC password in {password_file}")
        auth = self.runtime / "vnc.passwd"
        auth.touch(mode=0o600)
        with auth.open("wb") as output:
            subprocess.run(["tigervncpasswd", "-f"], input=(password+"\n").encode(), stdout=output, check=True)
        authority = self.runtime / "Xauthority"
        authority.touch(mode=0o600)
        subprocess.run(["xauth", "-f", str(authority), "add", f":{display}", ".", secrets.token_hex(16)], check=True)
        self.env.update({
            "DISPLAY": f":{display}", "XAUTHORITY": str(authority), "XDG_RUNTIME_DIR": str(self.runtime),
            "QT_QPA_PLATFORM": "xcb", "MESA_LOADER_DRIVER_OVERRIDE": "swrast", "GALLIUM_DRIVER": "llvmpipe",
            "LIBGL_ALWAYS_SOFTWARE": "1", "LP_NUM_THREADS": "4", "HEADLESS": "False", "USE_RVIZ": "True",
            "SHOW_MISSION_PANEL": "True", "WAREHOUSE_BROWSER_AGENT_FILE": str(self.runtime/"agent.json"),
        })
        self.env.pop("WAYLAND_DISPLAY", None)
        self.env.pop("DBUS_SESSION_BUS_ADDRESS", None)
        vnc_socket = self.runtime / "vnc.sock"
        self.spawn("desktop", ["Xtigervnc", f":{display}", "-geometry", "1600x1000", "-depth", "24",
                                "-desktop", "Warehouse demo", "-localhost", "-nolisten", "tcp",
                                "-rfbport", "-1", "-rfbunixpath", str(vnc_socket), "-rfbunixmode", "0600",
                                "-SecurityTypes", "VncAuth", "-rfbauth", str(auth), "-auth", str(authority),
                                "-AlwaysShared", "-FrameRate", "20"])
        self.wait_ready(lambda: subprocess.run(["xdpyinfo"], env=self.env, stdout=subprocess.DEVNULL,
                                               stderr=subprocess.DEVNULL).returncode == 0, "desktop")
        self.spawn("window-manager", ["openbox", "--sm-disable"])
        self.spawn("taskbar", ["tint2"])
        self.spawn("gateway", ["websockify", "--web", "/usr/share/novnc", "--unix-target", str(vnc_socket),
                               f"127.0.0.1:{self.port}"])
        self.wait_ready(lambda: url_ready(f"http://127.0.0.1:{self.port}/vnc.html"), "gateway")
        if not url_ready("http://127.0.0.1:11434/api/tags"):
            self.spawn("model", ["bash", "scripts/run_model.sh"])
            self.wait_ready(lambda: url_ready("http://127.0.0.1:11434/api/tags"), "model")
        if subprocess.run(["ollama", "show", "qwen3.5:4b"], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode != 0:
            raise RuntimeError("The local model is missing. Run ollama pull qwen3.5:4b before starting the browser demo.")
        self.spawn("simulation", ["bash", "scripts/run_sim.sh"])
        self.spawn("commands", ["xterm", "-T", "Warehouse commands", "-fa", "DejaVu Sans Mono", "-fs", "11",
                                "-bg", "#132238", "-fg", "#d9e7f6", "-cr", "#22d3ee", "-sb", "-sl", "5000",
                                "-e", "python3", "-B", "-m", "warehouse_agent.remote_desktop", "agent"])
        session = {"pid": os.getpid(), "process_token": process_token(os.getpid()), "port": self.port,
                   "display": f":{display}", "authority": str(authority), "vnc_socket": str(vnc_socket),
                   "password_file": str(password_file), "started": datetime.now(timezone.utc).isoformat(),
                   "url": f"http://localhost:{self.port}/vnc.html?autoconnect=1&resize=scale&reconnect=1"}
        private_write(SESSION, json.dumps(session, indent=2)+"\n")
        show_info(session, reveal_password=False)

    def arrange(self):
        result = subprocess.run(["wmctrl", "-lp"], env=self.env, capture_output=True, text=True)
        for line in result.stdout.splitlines():
            fields = line.split(None, 4)
            if len(fields) < 5 or fields[0] in self.arranged:
                continue
            window, title = fields[0], fields[4]
            # Gazebo applies its saved QML size after creating the X window.
            first_seen = self.window_seen.setdefault(window, time.monotonic())
            if time.monotonic()-first_seen < 6:
                continue
            if title == "Gazebo Sim":
                geometry = "0,0,0,990,704"
            elif title == "Warehouse · Mission view":
                geometry = "0,1000,0,594,720"
            elif title == "Warehouse commands":
                geometry = "0,0,746,1594,215"
            elif title.endswith(" - RViz"):
                geometry = "0,0,0,1594,948"
            else:
                continue
            subprocess.run(["wmctrl", "-ir", window, "-b", "remove,maximized_vert,maximized_horz"], env=self.env, check=True)
            subprocess.run(["wmctrl", "-ir", window, "-e", geometry], env=self.env, check=True)
            if title.endswith(" - RViz"):
                subprocess.run(["xdotool", "windowminimize", window], env=self.env, check=True)
            self.arranged.add(window)

    def run(self):
        self.start()
        while self.running:
            if self.children["commands"].poll() is not None:
                break
            for name, process in self.children.items():
                if name != "commands" and process.poll() is not None:
                    raise RuntimeError(f"{name} exited; see {DIRECTORY / (name+'.log')}")
            self.arrange()
            time.sleep(0.4)

    def stop_child(self, name, sig=signal.SIGTERM, timeout=4):
        process = self.children.get(name)
        if process is None:
            return
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()

    def close(self):
        # Let the agent acknowledge cancellation while Nav2 is still available.
        if self.runtime:
            try:
                agent = json.loads((self.runtime/"agent.json").read_text())
                if process_token(agent["pid"]) == agent["process_token"]:
                    os.kill(agent["pid"], signal.SIGINT)
                    deadline = time.monotonic()+12
                    while process_token(agent["pid"]) == agent["process_token"] and time.monotonic() < deadline:
                        time.sleep(0.1)
            except (OSError, ValueError, KeyError):
                pass
        # ROS launch should receive the first interrupt, not every child at once.
        simulation = self.children.get("simulation")
        if simulation and simulation.poll() is None:
            simulation.send_signal(signal.SIGINT)
            try:
                simulation.wait(timeout=20)
            except subprocess.TimeoutExpired:
                pass
        for name in ("commands", "simulation", "model", "gateway", "taskbar", "window-manager", "desktop"):
            self.stop_child(name)
        if self.runtime:
            shutil.rmtree(self.runtime)
        session = running_session()
        if session and session["pid"] == os.getpid():
            SESSION.unlink(missing_ok=True)
        for stream in self.logs:
            stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "info", "stop", "agent"), nargs="?", default="start")
    parser.add_argument("--port", type=int, default=6080)
    args = parser.parse_args()
    if args.action == "agent":
        path = Path(os.environ["WAREHOUSE_BROWSER_AGENT_FILE"])
        private_write(path, json.dumps({"pid": os.getpid(), "process_token": process_token(os.getpid())}))
        os.execv("/bin/bash", ["bash", str(ROOT/"scripts/run_agent.sh")])
    if args.action in {"info", "stop"}:
        session = running_session()
        if session is None:
            print("Browser demo is stopped.")
            return 0
        if args.action == "info":
            show_info(session)
        else:
            os.kill(session["pid"], signal.SIGTERM)
            print("Stopping the browser demo…", flush=True)
            deadline = time.monotonic()+45
            while process_token(session["pid"]) == session["process_token"] and time.monotonic() < deadline:
                time.sleep(0.2)
            if process_token(session["pid"]) == session["process_token"]:
                print(f"Shutdown is taking longer than expected; inspect {DIRECTORY}.")
                return 1
            print("Browser demo stopped.", flush=True)
        return 0
    if not 1024 <= args.port <= 65535:
        parser.error("--port must be between 1024 and 65535")
    DIRECTORY.mkdir(parents=True, exist_ok=True, mode=0o700)
    DIRECTORY.chmod(0o700)
    with (DIRECTORY/"launcher.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("A browser demo is already running. Use scripts/wsl.sh browser-info.")
            return 2
        desktop = Desktop(args.port)
        def stop(*_):
            desktop.running = False
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGHUP, stop)
        try:
            desktop.run()
        except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
            print(f"Browser demo failed: {exc}", flush=True)
            return 1
        finally:
            desktop.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
