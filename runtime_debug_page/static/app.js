// Set iframe URLs using the current hostname so the page works both on the Pi
// and from another machine on the same network.
(function () {
  const host = window.location.hostname;

  document.getElementById("rqt-frame").src =
    "http://" + host + ":6080/vnc.html?autoconnect=true&resize=scale";

  document.getElementById("rosa-terminal").src =
    "http://" + host + ":7681";
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

    tickId.textContent = data.tick_id;
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
