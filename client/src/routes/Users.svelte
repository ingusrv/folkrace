<script lang="ts">
  import Header from "../lib/Header.svelte";
  import { auth } from "../stores";
  import type { User } from "../types";
  import { Role } from "../types";
  import { API_URL } from "../config";

  let token = $auth.user.token;
  let users: User[] = [];

  fetch(`${API_URL}/users`, {
    method: "get",
    headers: {
      Authorization: `Bearer ${token}`,
    },
  })
    .then((res) => res.json())
    .then((data) => {
      console.log(data);
      users = data;
    });

  function editUser(userId: string) {
    console.log("editing user", userId);
  }

  function deleteUser(userId: string) {
    console.log("deleting user", userId);
    fetch(`${API_URL}/user/${userId}`, {
      method: "delete",
      headers: {
        Authorization: `Bearer ${token}`,
      },
    }).then(async (res) => {
      if (res.status === 200) {
        const data = await res.json();
        console.log(data);
        users = users.filter((user) => user._id !== userId);
      } else {
        console.log("error deleting user");
      }
    });
  }

  let newUserDialog: HTMLDialogElement;
  function addUser(e: SubmitEvent) {
    e.preventDefault();
    console.log("adding user...");
    const formData = new FormData(e.target as HTMLFormElement);
    console.log(formData.get("username"), formData.get("password"));

    fetch(`${API_URL}/user`, {
      method: "post",
      mode: "cors",
      headers: {
        "Content-Type": "application/json",
        Authorization: `Bearer ${token}`,
      },
      body: JSON.stringify({
        username: formData.get("username"),
        password: formData.get("password"),
        admin: formData.get("admin"),
      }),
    }).then(async (res) => {
      if (res.status === 201) {
        const response = await res.json();
        console.log("added user");
        console.log(response);
        users = [...users, response];
        newUserDialog.close();
      } else {
        console.log("error adding user");
        const response = await res.json();
        console.log(response.message);
      }
    });
  }
</script>

<Header />
<main>
  <dialog
    class="surface-container on-surface-container"
    bind:this={newUserDialog}
  >
    <h2 class="dialog-title">Pievienot jaunu lietotāju</h2>
    <form method="post" class="add-user-form" on:submit={addUser}>
      <label for="username">Lietotājvārds</label>
      <input
        class="input on-surface-container border-on-surface-container"
        type="text"
        name="username"
        id="username"
      />
      <label for="password">Parole</label>
      <input
        class="input on-surface-container border-on-surface-container"
        type="password"
        name="password"
        id="password"
      />
      <label for="admin" class="inline">Administrators</label>
      <input type="checkbox" name="admin" id="admin" />
      <button class="button success on-success width-full" type="submit">
        Pievienot
      </button>
    </form>
    <form method="dialog" id="close-view-data-dialog">
      <button class="button primary on-primary" type="submit">Atcelt</button>
    </form>
  </dialog>

  <div class="width-full">
    <button
      class="button primary on-primary"
      on:click={() => newUserDialog.showModal()}
    >
      Pievienot lietotāju
    </button>
  </div>

  <div class="panel surface-container on-surface-container">
    <table class="data-table width-full">
      <thead>
        <tr>
          <th>Nr.</th>
          <th>Lietotājvārds</th>
          <th>Loma</th>
          <th>Izveides datums</th>
          <th>Darbības</th>
        </tr>
      </thead>
      <tbody>
        {#each users as user, i}
          <tr>
            <td>{i + 1}</td>
            <td>{user.username}</td>
            <td>{Role[user.role]}</td>
            <td>{new Date(user.createdAt).toLocaleString("lv")}</td>
            <td>
              <div class="actions-container">
                <button
                  class="button secondary on-secondary"
                  on:click={() => editUser(user._id)}
                >
                  Rediģēt
                </button>
                <button
                  class="button danger on-danger"
                  on:click={() => deleteUser(user._id)}
                >
                  Izdzēst
                </button>
              </div>
            </td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
</main>

<style>
  .add-user-form {
    font-size: 1.5rem;
    margin-bottom: 1rem;
  }

  .add-user-form * {
    display: block;
  }

  .add-user-form input {
    width: 100%;
    margin-bottom: 1rem;
  }

  .add-user-form input[type="checkbox"] {
    display: inline;
    width: 1.25rem;
    height: 1.25rem;
  }
</style>
