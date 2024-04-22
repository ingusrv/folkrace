<script lang="ts">
  import { Route, Router } from "svelte-routing";
  import Data from "./routes/Data.svelte";
  import Login from "./routes/Login.svelte";
  import Robots from "./routes/Robots.svelte";
  import Users from "./routes/Users.svelte";
  import Logout from "./routes/Logout.svelte";
  import { auth } from "./stores";
  import type { User } from "./types";
  import AuthRoute from "./lib/AuthRoute.svelte";
  export let url = "";

  const cachedUser = localStorage.getItem("user");
  if (cachedUser) {
    const parsedUser = JSON.parse(cachedUser) as User;
    console.log("cached user", parsedUser);
    auth.set({
      authenticated: true,
      user: parsedUser,
    });
  }
</script>

<Router {url}>
  <Route path="/login">
    <Login />
  </Route>
  <AuthRoute path="/logout">
    <Logout />
  </AuthRoute>
  <AuthRoute path="/">
    <Data />
  </AuthRoute>
  <AuthRoute path="/data">
    <Data />
  </AuthRoute>
  <AuthRoute path="/robots">
    <Robots />
  </AuthRoute>
  <AuthRoute path="/users">
    <Users />
  </AuthRoute>
</Router>
