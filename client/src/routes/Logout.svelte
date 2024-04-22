<script lang="ts">
  import { navigate } from "svelte-routing";
  import { API_URL } from "../config";
  import { auth } from "../stores";

  let token = $auth.user.token;
  let message: string;

  fetch(`${API_URL}/auth/logout`, {
    method: "get",
    mode: "cors",
    headers: {
      "Content-Type": "application/json",
      Authorization: `Bearer ${token}`,
    },
  }).then(async (res) => {
    if (res.status === 200) {
      message = "logged out successfully";
      localStorage.setItem("user", "");
      navigate("/login", { replace: true });
    } else {
      const response = await res.json();
      message = response.message;
    }
  });
</script>

<h1>logging out of folkrace...</h1>
<div>{message}</div>
