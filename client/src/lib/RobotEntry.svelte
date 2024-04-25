<script lang="ts">
  import type { Robot } from "../types";
  import { RobotState } from "../types";
  import { MessageType } from "../types";

  export let number: number;
  export let robot: Robot;
  export let ws: WebSocket;

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
          delay: robot.delay,
        }),
      );
    }
  }
</script>

<tr>
  <td>{number}</td>
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
      <button class="button secondary on-secondary">Iestatījumi</button>
      <button class="button danger on-danger">Izdzēst</button>
    </div>
  </td>
</tr>

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
    width: 4rem;
  }
</style>
