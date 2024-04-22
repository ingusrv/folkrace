<script lang="ts">
  import { link, Link } from "svelte-routing";
</script>

<header class="surface-container on-surface-container">
  <nav>
    <ul>
      <li>
        <Link to="/data" let:active>
          <span class:active> Braucienu dati </span>
        </Link>
      </li>
      <li>
        <Link to="/robots" let:active>
          <span class:active> Robotu saraksts </span>
        </Link>
      </li>
      <li>
        <Link to="/users" let:active>
          <span class:active> Lietotāji </span>
        </Link>
      </li>
      <li>
        <a class="button danger on-danger" href="/logout" use:link>Iziet</a>
      </li>
    </ul>
  </nav>
</header>

<style>
  header {
    margin-bottom: 4rem;
    font-size: 1.25rem;
  }

  header ul {
    list-style-type: none;
    margin: 0;
    padding: 0;
    display: flex;
    flex-direction: row;
  }

  span {
    text-decoration: none;
    color: white;
    display: inline-block;
    padding: 1em;
  }

  header li:last-child {
    padding: 1em;
    margin-left: auto;
  }

  header a {
    text-decoration: none;
    color: inherit;
  }

  .active {
    border-bottom: 5px solid var(--primary);
    border-radius: 5px;
  }
</style>
