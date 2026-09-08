# Watch and operate the warehouse through SSH

The browser mode streams the real Gazebo, RViz, mission-panel and command-terminal windows using noVNC. The simulator runs inside Ubuntu-24.04 on a private virtual desktop. Your browser receives the screen and sends keyboard/mouse input through an SSH tunnel.

## Connect from your computer

In your **remote WSL shell**, start the browser desktop:

```bash
cd /path/to/ros-llm-ops
bash scripts/wsl.sh browser
```

Leave this command running. It starts the desktop, local model server when needed, simulator, mission panel and command terminal. If a browser session is already running, reuse it.

In a **second terminal on your local computer**, forward port 6080 using your usual SSH destination:

```bash
ssh -N -L 6080:127.0.0.1:6080 YOUR_SSH_HOST
```

Replace `YOUR_SSH_HOST` with your normal SSH alias or `user@host`. Keep any existing `-p` or `-J` options you use to reach this machine. The forwarding process stays running without opening a shell. If your remote editor provides a Ports panel, forwarding remote port **6080** serves the same purpose.

Open this URL in your local browser:

[Warehouse browser desktop](http://localhost:6080/vnc.html?autoconnect=1&resize=scale&reconnect=1)

To obtain the VNC password, run this in another **remote WSL shell**:

```bash
bash scripts/wsl.sh browser-info
```

Enter the displayed password at the browser login. The launcher creates one random password and retains it across restarts in a file readable only by your runtime user. It does not put the password into the URL.

## Run the visible pipeline

Wait until the terminal inside the browser says `Nav2 and localization ready`. Click that terminal at the bottom and enter:

```text
Deliver all three parcels
```

You can follow the interpreted request and navigation/cargo events in the terminal, see the robot and parcels move in Gazebo, and watch the current goal and remaining stops in the mission panel. Click **RViz** in the bottom taskbar to bring up the navigation map and sensor view. Minimize RViz to return to the overview; closing RViz shuts down the stock Nav2 launch.

The existing `/status`, `/pause` and `/resume` commands work in the browser terminal. `/quit` ends the agent and shuts down this browser desktop. The noVNC toolbar on the left provides fullscreen and keyboard/clipboard controls. Browser-side scaling fits the 1600 × 1000 desktop into your window.

Closing the browser tab or disconnecting the SSH forwarding tunnel leaves the running demo intact. To return, reconnect the tunnel and reopen the page. Keep the remote launcher terminal open for the session; a hangup delivered to the launcher triggers cleanup.

## Stop or restart

From the remote source workspace:

```bash
bash scripts/wsl.sh browser-stop
bash scripts/wsl.sh browser
```

The stop command waits for cleanup. Restarting returns the robot to HOME and creates fresh logical parcel state. Only one warehouse simulator and one mission agent may run on ROS domain 42. Stop a separately launched local demo before switching to browser mode.

If local port 6080 is occupied, choose another **local** port:

```bash
ssh -N -L 16080:127.0.0.1:6080 YOUR_SSH_HOST
```

Then open `http://localhost:16080/vnc.html?autoconnect=1&resize=scale`. If the remote port is occupied, start the browser launcher with `--port 6081` and forward that port instead.

## Prepared setup and validation

Installed in Ubuntu-24.04: TigerVNC, noVNC/websockify, Openbox, tint2, xterm and the small X11 utilities used to arrange the windows. For another prepared ROS machine, `bash scripts/install_browser.sh --apply` installs these dependencies inside Ubuntu-24.04.

The HTTP/WebSocket gateway binds to **127.0.0.1 only**. TigerVNC uses a private Unix socket with VNC password authentication, and X access uses a per-session authority file. The configured gateway does not publish a desktop port on the LAN. Logs are under the runtime copy's `artifacts/browser/`; `scripts/wsl.sh logs` excludes the password, session metadata and temporary browser-test dependencies.

Qwen uses the available GPU for local inference. The virtual desktop uses Mesa software graphics so it can render without a Windows desktop session. This is suitable for watching the warehouse demo; frame rate depends on available CPU and connection bandwidth.

Validation on 8 September 2026:

- HTTP access succeeded from the source WSL distribution used by SSH.
- An actual Chromium browser authenticated to noVNC and displayed live desktop frames.
- A delivery instruction entered through the browser keyboard reached the mission supervisor.
- All three parcels were delivered across six successful navigation goals in **63.171 seconds after intent acceptance**; the browser reported no JavaScript errors.
- The stop command removed the owned simulator, terminal, virtual desktop, gateway and model processes.

See the [results](evidence/browser/browser-test.json) and [mission journal](evidence/browser/delivery.jsonl). Browser desktop captures containing local paths are excluded from the public repository; the [visual guide](stage1-visuals.md) links clean application captures. These are development checks, not a reliability or streaming-performance benchmark. The existing native ROS/Gazebo shutdown diagnostics remain documented in the [runbook](mvp-runbook.md#remaining-shutdown-issue).

Primary references: [noVNC deployment and URL options](https://novnc.com/noVNC/docs/EMBEDDING.html), [TigerVNC virtual server and access options](https://tigervnc.org/doc/Xvnc.html), [websockify](https://github.com/novnc/websockify), [OpenSSH port forwarding](https://man.openbsd.org/ssh).
