<script lang="ts">
  import Header from "../lib/Header.svelte";
  import { API_URL } from "../config";
  import type { Robot } from "../types";
  import { MessageType, RobotState } from "../types";
  import { auth } from "../stores";
  import { onMount } from "svelte";

  onMount(() => console.log("robots mounted"));

  let token = $auth.user.token;
  let robots: Robot[] = [];
  let websocket_opened = false;
  let robots_loaded = false;

  fetch(`${API_URL}/robots`, {
    method: "get",
    headers: {
      Authorization: `Bearer ${token}`,
    },
  })
    .then((data) => data.json())
    .then((data) => {
      console.log(data);
      data.forEach((r: Robot) => {
        r.delay = 0;
        r.state = RobotState.disconnected;
      });
      robots = data;
      robots_loaded = true;
    });

  function getStatusMessage(state: RobotState) {
    switch (state) {
      case RobotState.disconnected:
        return "Nav savienots";
      case RobotState.connected:
        return "Savienots";
      case RobotState.running:
        return "Programma startēta";
      case RobotState.error:
        return "Notika kļūda";
      default:
        return "Nezināms";
    }
  }

  const ws = new WebSocket(`${API_URL}/user?token=${token}`);
  ws.addEventListener("open", (e) => {
    console.log("ws opened");
    websocket_opened = true;
  });

  ws.addEventListener("message", (e) => {
    console.log("raw data", e.data);
    const msg = JSON.parse(e.data);
    console.log("new message", msg);

    switch (msg.type) {
      case MessageType.status:
        robots.map((robot) => {
          if (robot._id === msg.robotId) {
            console.log("new robot state for robot", robot.name, msg.state);
            robot.state = msg.state;
          }
        });
        robots = robots;
        break;
      default:
        console.log("dont know that type", msg.type);
        break;
    }
  });

  function toggleRobotProgram(robot: Robot) {
    if (robot.state === RobotState.running) {
      console.log("stopping program for robot", robot._id);
      ws.send(
        JSON.stringify({
          type: MessageType.stop,
          robotId: robot._id,
        }),
      );
    } else {
      console.log("starting robot program for", robot._id);
      console.log("start delay is", robot.delay);
      ws.send(
        JSON.stringify({
          type: MessageType.start,
          robotId: robot._id,
        }),
      );
    }
    robots = robots;
  }

  $: if (websocket_opened && robots_loaded) queryRobotStatus();
  function queryRobotStatus() {
    robots.forEach((r: Robot) => {
      console.log(r.name);
      ws.send(
        JSON.stringify({
          type: MessageType.status,
          robotId: r._id,
        }),
      );
    });
  }
</script>

<Header />
<main>
  <div class="width-full">
    <button class="button primary on-primary">Pievienot robotu</button>
  </div>

  <div class="panel surface-container on-surface-container">
    <table class="data-table width-full">
      <thead>
        <tr>
          <th>Nr.</th>
          <th>Robota nosaukums</th>
          <th>Izveides datums</th>
          <th>Savienošanās atslēga</th>
          <th>Robota statuss</th>
          <th>Starta delay, s</th>
          <th>Darbības</th>
        </tr>
      </thead>
      <tbody>
        {#each robots as robot, i}
          <tr>
            <td>{i + 1}</td>
            <td>{robot.name}</td>
            <td>{new Date(robot.createdAt).toLocaleString("lv")}</td>
            <td>
              <button
                class="button secondary on-secondary"
                on:click={() => navigator.clipboard.writeText(robot.token)}
              >
                kopēt
              </button>
              <!--<HiddenInput token={robot.token} />-->
            </td>
            <td>
              <span
                class="pill status"
                class:online={robot.state === RobotState.connected ||
                  robot.state === RobotState.running}
              >
                {getStatusMessage(robot.state)}
              </span>
            </td>
            <td>
              <input
                type="number"
                class="input surface-container on-surface-container delay"
                bind:value={robot.delay}
              />
            </td>
            <td>
              <div class="actions-container">
                <button
                  class="button secondary on-secondary"
                  on:click={() => toggleRobotProgram(robot)}
                >
                  {robot.state === RobotState.running
                    ? "Beigt robota programmu"
                    : "Sākt robota programmu"}
                </button>
                <button class="button secondary on-secondary"
                  >Iestatījumi</button
                >
                <button class="button danger on-danger">Izdzēst</button>
              </div>
            </td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
</main>

<style>
  .status {
    background-color: var(--danger);
    color: var(--on-danger);
  }

  .online {
    background-color: var(--success);
    color: var(--on-success);
  }

  .delay {
    width: 3rem;
  }
</style>
