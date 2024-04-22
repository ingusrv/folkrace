<script lang="ts">
  export let token: string = "";
  let hidden = true;
</script>

<span>
  <input type={hidden ? "password" : "text"} value={token} readonly />
  <button class="button surface on-surface" on:click={() => (hidden = !hidden)}>
    {hidden ? "show" : "hide"}
  </button>
</span>

<style>
  input {
    /*width: 10.2rem;*/
    border: none;
    cursor: default;
    overflow-x: scroll;
  }

  input:focus {
    outline: none;
  }
</style>
