import { writable } from "svelte/store";
import type { User } from "./types";
import { Role } from "./types";

const emptyUser: User = {
  _id: "",
  username: "",
  token: "",
  role: Role.user,
  createdAt: new Date()
}

export const auth = writable({
  authenticated: false,
  user: emptyUser
});
