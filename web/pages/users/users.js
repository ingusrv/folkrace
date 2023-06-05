import Notification from "../notification.js";

const addUser = document.querySelector("#add-user");
const addUserDialog = document.querySelector("#add-user-dialog");
const userTable = document.querySelector("#user-table").querySelector("tbody");

function loadUsers() {
    userTable.replaceChildren();
    fetch("/api/users").then(async (res) => {
        if (res.status >= 400) {
            new Notification({ type: "error", text: "Notika kļūda ielādējot lietotāju datus!" });
            return;
        }

        const body = await res.json();
        console.log(body);

        body.data.forEach((user, i) => {
            const row = document.createElement("tr");

            // dati
            const number = document.createElement("td");
            number.textContent = ++i;
            const username = document.createElement("td");
            username.textContent = user.username;
            const isAdmin = document.createElement("td");
            isAdmin.textContent = user.admin;
            const createdAt = document.createElement("td");
            createdAt.textContent = user.createdAt;

            // darbības
            const actions = document.createElement("td");
            const editButton = document.createElement("button");
            editButton.textContent = "Rediģēt";
            editButton.classList.add("button", "secondary");
            const deleteButton = document.createElement("button");
            deleteButton.textContent = "Noņemt";
            deleteButton.classList.add("button", "danger");

            actions.append(editButton, deleteButton);

            // pogu funkcionalitāte
            editButton.addEventListener("click", () => { });
            deleteButton.addEventListener("click", () => {
                fetch(`/api/user/${user.username}`, {
                    method: "DELETE",
                    mode: "same-origin",
                }).then(async (res) => {
                    const data = await res.json();
                    console.log(data);

                    if (res.status >= 400) {
                        new Notification({ type: "error", text: data.message });
                        return;
                    }

                    new Notification({ type: "success", text: data.message });
                    // nav optimāli pārlādēt visu sarakstu, bet pagaidām strādā
                    loadUsers();
                });
            });

            row.append(number, username, isAdmin, createdAt, actions);
            userTable.appendChild(row);
        });
    });
}

addUser.addEventListener("click", (e) => {
    addUserDialog.showModal();
    const form = addUserDialog.querySelector("#add-user-form");
    form.addEventListener("submit", (e) => {
        e.preventDefault();

        const username = form.querySelector("#username");
        const password = form.querySelector("#password");
        const isAdmin = form.querySelector("#isAdmin");

        const user = {
            username: username.value,
            password: password.value,
            admin: isAdmin.checked
        };

        fetch("/api/user", {
            method: "POST",
            mode: "same-origin",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(user)
        }).then(async (res) => {
            const data = await res.json();
            console.log(data);

            if (res.status >= 400) {
                new Notification({ type: "error", text: data.message });
                return;
            }

            new Notification({ type: "success", text: data.message });
            // nav optimāli pārlādēt visu sarakstu, bet pagaidām strādā
            loadUsers();
        });

        username.value = "";
        password.value = "";
        isAdmin.checked = false;
        addUserDialog.close();
    });
});

loadUsers();