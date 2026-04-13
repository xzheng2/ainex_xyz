// ── rqt iframe URL ────────────────────────────────────────────────────────────
(function () {
  const host = window.location.hostname;
  document.getElementById("rqt-frame").src =
    "http://" + host + ":6080/vnc.html?autoconnect=true&resize=scale";
})();

// ── Tick polling ──────────────────────────────────────────────────────────────

async function refreshTick() {
  const tickId     = document.getElementById("tick-id");
  const tickStatus = document.getElementById("tick-status");
  const tickAge    = document.getElementById("tick-age");

  try {
    const res  = await fetch("/api/current_tick", { cache: "no-store" });
    const data = await res.json();

    tickStatus.classList.remove("status-warn", "status-error");

    if (!data.ok) {
      tickId.textContent     = "?";
      tickAge.textContent    = "?";
      tickStatus.textContent = data.error || data.status || "unavailable";
      tickStatus.classList.add("status-error");
      return;
    }

    tickId.textContent  = data.tick_id;
    tickAge.textContent =
      data.age_seconds == null ? "?" : data.age_seconds.toFixed(1) + "s";

    if (data.stale) {
      tickStatus.textContent = "paused/stale";
      tickStatus.classList.add("status-warn");
    } else {
      tickStatus.textContent = "live";
    }
  } catch (err) {
    tickId.textContent     = "?";
    tickAge.textContent    = "?";
    tickStatus.textContent = "api error";
    tickStatus.classList.add("status-error");
  }
}

setInterval(refreshTick, 500);
refreshTick();

// ── ROSA chat panel ───────────────────────────────────────────────────────────
// Connects directly to the ttyd WebSocket proxy from the parent page.
// No iframe → no cross-origin focus issues. Keyboard input works natively.

(function () {
  const output = document.getElementById("rosa-output");
  const input  = document.getElementById("rosa-input");
  const btn    = document.getElementById("rosa-send");

  // Strip ANSI escape sequences so raw terminal output is readable as plain text.
  function stripAnsi(str) {
    return str
      .replace(/\x1b\[[0-9;]*[A-Za-z]/g, "")   // CSI sequences (colors, cursor)
      .replace(/\x1b[()][0-9A-Za-z]/g, "")      // character set
      .replace(/\x1b./g, "");                    // any other ESC + char
  }

  function appendOutput(text) {
    // Normalize line endings: PTY sends \r\n; bare \r would overwrite the line.
    const cleaned = stripAnsi(text).replace(/\r\n/g, "\n").replace(/\r/g, "\n");
    output.textContent += cleaned;
    output.scrollTop = output.scrollHeight;
  }

  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const wsUrl = proto + "//" + location.host + "/terminal/ws";

  let ws = null;

  function connect() {
    appendOutput("[Connecting to ROSA agent…]\n");
    ws = new WebSocket(wsUrl, ["tty"]);

    ws.onopen = function () {
      // Tell ttyd the terminal size
      ws.send("1" + JSON.stringify({ cols: 90, rows: 20 }));
      ws.send("4"); // ping
      appendOutput("[Connected]\n");
      input.disabled = false;
      btn.disabled   = false;
      input.focus();
    };

    function handleMsg(str) {
      if (!str.length) return;
      const cmd  = str[0];
      const body = str.slice(1);
      if (cmd === "0") appendOutput(body); // terminal output
      // cmd "1" = window title, "2" = preferences — ignore
    }

    ws.binaryType = "arraybuffer"; // receive binary as ArrayBuffer, not Blob

    ws.onmessage = function (e) {
      if (e.data instanceof ArrayBuffer) {
        handleMsg(new TextDecoder().decode(e.data));
      } else if (typeof e.data === "string") {
        handleMsg(e.data);
      }
    };

    ws.onerror = function () {
      appendOutput("\n[WebSocket error — ROSA agent may not be running]\n");
    };

    ws.onclose = function () {
      appendOutput("\n[Connection closed — reload page to reconnect]\n");
      input.disabled = true;
      btn.disabled   = true;
    };
  }

  function sendInput() {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    const text = input.value;
    if (!text) return;
    input.value = "";
    // Echo the sent text immediately so it appears in the output area.
    // The PTY also echoes it back, but that round-trip can be slow.
    appendOutput(text + "\n");
    ws.send("0" + text + "\r"); // "0" = INPUT command; \r = Enter
  }

  input.addEventListener("keydown", function (e) {
    if (e.key === "Enter") sendInput();
  });

  btn.addEventListener("click", sendInput);

  // Start disabled until WebSocket connects
  input.disabled = true;
  btn.disabled   = true;

  connect();
})();
