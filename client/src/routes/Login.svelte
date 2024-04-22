<script lang="ts">
  import { navigate } from "svelte-routing";
  import { API_URL } from "../config";
  import { auth } from "../stores";

  let errorMessage: string;
  async function login(e: SubmitEvent) {
    e.preventDefault();
    console.log("logging in...");
    const formData = new FormData(e.target as HTMLFormElement);
    console.log(formData.get("username"), formData.get("password"));

    const res = await fetch(`${API_URL}/auth/login`, {
      method: "post",
      mode: "cors",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        username: formData.get("username"),
        password: formData.get("password"),
      }),
    });

    if (res.status === 200) {
      const user = await res.json();
      console.log("logged in");
      console.log(user);
      localStorage.setItem("user", JSON.stringify(user));
      auth.set({
        authenticated: true,
        user: user,
      });
      navigate("/data");
    } else {
      console.log("error logging in");
      const response = await res.json();
      console.log(response.message);
      errorMessage = response.message;
    }
  }
</script>

<main>
  <div class="login-container surface-container on-surface-container">
    <form method="post" on:submit={login}>
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
      {#if errorMessage}
        <div class="error-message danger on-danger">
          {errorMessage}
        </div>
      {/if}
      <button class="button primary on-primary" type="submit">
        Pieslēgties
      </button>
    </form>
  </div>
</main>

<style>
  main {
    height: 100vh;
    width: 100vw;
    overflow: hidden;
    display: grid;
    place-items: center;
  }

  .login-container {
    width: 21%;
    min-width: fit-content;
    padding: 1.5rem;
    font-size: 1.5rem;
    border-radius: 12px;
  }

  .login-container * {
    display: block;
  }

  .login-container input {
    width: 100%;
    height: 3rem;
    margin-bottom: 2rem;
  }

  .login-container button {
    width: 100%;
    height: 3rem;
  }

  .error-message {
    padding: 0.2rem 0.3rem;
    border-radius: 6px;
    margin-bottom: 2rem;
    text-align: center;
  }
</style>
