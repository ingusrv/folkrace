<script lang="ts">
  import Redirect from "./Redirect.svelte";
  import { Route } from "svelte-routing";
  import { auth } from "../stores";
  export let path: string;
</script>

<Route {path}>
  {#if $auth.authenticated}
    <slot />
  {:else}
    <Redirect to="/login" />
  {/if}
</Route>
