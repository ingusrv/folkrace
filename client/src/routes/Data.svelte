<script lang="ts">
  import Header from "../lib/Header.svelte";
  import { auth } from "../stores";
  import type { DriveData } from "../types";
  import { API_URL } from "../config";

  let driveData: DriveData[] = [];
  let token = $auth.user.token;

  fetch(`${API_URL}/driveData`, {
    method: "get",
    headers: {
      Authorization: `Bearer ${token}`,
    },
  })
    .then((res) => res.json())
    .then((data) => {
      console.log("new drive data");
      console.log(data);
      driveData = data;
    });

  function deleteData(dataId: string) {
    console.log("deleting drive data", dataId);
    fetch(`${API_URL}/driveData/${dataId}`, {
      method: "delete",
      headers: {
        Authorization: `Bearer ${token}`,
      },
    }).then(async (res) => {
      if (res.status === 200) {
        const data = await res.json();
        console.log(data);
        driveData = driveData.filter((data) => data._id !== dataId);
      } else {
        console.log("error deleting drive data");
      }
    });
  }

  let viewDataDialog: HTMLDialogElement;
  function showDataDialog(dataId: string) {
    const data = driveData.find((data) => data._id === dataId);
    console.log("getting data info", data);
    //viewDataDialog.showModal();
  }
</script>

<Header />
<main>
  <dialog
    class="surface-container on-surface-container width-half"
    id="chart-dialog"
  >
    <h2 class="dialog-title">Datu salīdzinājums</h2>
    <div class="chart-container">
      <canvas id="comparison-chart"></canvas>
    </div>
    <form method="dialog" id="close-chart-dialog">
      <button class="button primary on-primary" type="submit">Aizvērt</button>
    </form>
  </dialog>
  <dialog
    class="surface-container on-surface-container width-half"
    id="view-data-dialog"
    bind:this={viewDataDialog}
  >
    <h2 class="dialog-title">No robota saņemtie dati</h2>
    <div id="text-window" class="text-window"></div>
    <form method="dialog" id="close-view-data-dialog">
      <button class="button primary on-primary" type="submit">Aizvērt</button>
    </form>
  </dialog>
  <div class="width-full">
    <button class="button primary on-primary">Salīdzināt datus</button>
  </div>
  <div class="panel surface-container on-surface-container">
    {#if driveData.length > 0}
      <table class="data-table width-full">
        <thead>
          <tr>
            <th></th>
            <th>Nr.</th>
            <th>Robota nosaukums</th>
            <th>Izveides datums</th>
            <th>Algoritms</th>
            <th>Versija</th>
            <th>Vidējais FPS</th>
            <th>Braukšanas laiks, s</th>
            <th>Darbības</th>
          </tr>
        </thead>
        <tbody>
          {#each driveData as data, i}
            <tr>
              <td></td>
              <td>{i + 1}</td>
              <td>{data.robot}</td>
              <td>{new Date(data.createdAt).toLocaleString("lv")}</td>
              <td>{data.algorithm}</td>
              <td>{data.version}</td>
              <td>{data.averageFps}</td>
              <td>{data.elapsedTime}</td>
              <td>
                <div class="actions-container">
                  <button
                    class="button secondary on-secondary"
                    on:click={() => showDataDialog(data._id)}
                  >
                    Skatīt
                  </button>
                  <button
                    class="button danger on-danger"
                    on:click={() => deleteData(data._id)}
                  >
                    Izdzēst
                  </button>
                </div>
              </td>
            </tr>
          {/each}
        </tbody>
      </table>
    {:else}
      <div class="no-data">Nav datu</div>
    {/if}
  </div>
</main>

<style>
  .no-data {
    text-align: center;
    font-size: 1.25rem;
  }
</style>
