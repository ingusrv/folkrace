import Notification from "../notification.js";

const addUser = document.querySelector("#add-user");
const cancelAddUser = document.querySelector("#cancel-add-user");
const addUserForm = document.querySelector("#add-user-form");
const removeUser = document.querySelector("#remove-user");
const cancelRemoveUser = document.querySelector("#cancel-remove-user");
const removeUserForm = document.querySelector("#confirm-remove-user");
let usersToDelete = [];
let checkboxes = undefined;

fetch("/api/v1/users").then(async (res) => {
    const table = document.querySelector("#user-table");
    const body = await res.json();
    console.log(body);

    // TODO: fix potential XSS
    // TODO: replace table with message if no data or error
    body.data.forEach((item, i) => {
        const tr = document.createElement("tr");
        tr.innerHTML = `<td><input class="remove-user-checkbox" type="checkbox" name="remove-user-checkbox" data-remove-user-checkbox data-username="${item.username}"></td><td>${++i}</td><td>${item.username}</td><td>${item.admin}</td><td>${item.createdAt}</td>`;
        table.appendChild(tr);
    });

    checkboxes = document.querySelectorAll("[data-remove-user-checkbox]");
    loadCheckboxes(checkboxes);
});

addUser.addEventListener("click", () => {
    document.querySelector("#dim-background").style.opacity = 0.8;
    document.querySelector("#dim-background").classList.toggle("hidden");
    document.querySelector("#add-user-prompt").classList.toggle("hidden");
});

cancelAddUser.addEventListener("click", () => {
    document.querySelector("#dim-background").style.opacity = 0.8;
    document.querySelector("#dim-background").classList.toggle("hidden");
    document.querySelector("#add-user-prompt").classList.toggle("hidden");
});

// TODO: make this more responsive
addUserForm.addEventListener("submit", (e) => {
    e.preventDefault();
    const user = {
        username: document.querySelector("#username").value,
        password: document.querySelector("#password").value,
        admin: document.querySelector("#isAdmin").checked
    };

    fetch("/api/v1/user", {
        method: "POST",
        mode: "same-origin",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify(user)
    }).then(async (res) => {
        const body = await res.json();
        console.log(body);
        if (res.status >= 400) {
            new Notification({ type: "error", text: body.message });
        }
        if (res.status === 201) {
            new Notification({ type: "success", text: body.message });
        }

        document.querySelector("#dim-background").style.opacity = 0.8;
        document.querySelector("#dim-background").classList.toggle("hidden");
        document.querySelector("#add-user-prompt").classList.toggle("hidden");
    });
});

removeUser.addEventListener("click", () => {
    document.querySelector("#dim-background").style.opacity = 0.8;
    document.querySelector("#dim-background").classList.toggle("hidden");
    document.querySelector("#remove-user-prompt").classList.toggle("hidden");
});

cancelRemoveUser.addEventListener("click", () => {
    document.querySelector("#dim-background").style.opacity = 0.8;
    document.querySelector("#dim-background").classList.toggle("hidden");
    document.querySelector("#remove-user-prompt").classList.toggle("hidden");
});

function loadCheckboxes(checkboxes) {
    checkboxes.forEach((checkbox) => {
        checkbox.addEventListener("click", (e) => {
            const username = e.target.dataset.username;
            console.log(username);
            if (!usersToDelete.includes(username)) {
                usersToDelete.push(username);
                return;
            }

            const index = usersToDelete.indexOf(username);
            usersToDelete.splice(index, 1);
        });
    });
}

removeUserForm.addEventListener("click", () => {
    console.log(usersToDelete);
    let removedCount = 0;
    let unableToRemoveCount = 0;
    usersToDelete.forEach((username) => {
        fetch(`/api/v1/user/${username}`, {
            method: "DELETE",
            mode: "same-origin",
        }).then(async (res) => {
            const body = await res.json();
            console.log(body);

            // this is bad
            if (res.status >= 400) {
                unableToRemoveCount++;
            }
            if (res.status === 200) {
                removedCount++;
            }

            if ((removedCount + unableToRemoveCount) === usersToDelete.length) {
                if (removedCount !== usersToDelete.length) {
                    new Notification({ type: "error", text: `${unableToRemoveCount} lietotāju(s) nevarēja noņemt!` });
                    return;
                }
                if (removedCount === 1) {
                    new Notification({ type: "success", text: `${removedCount} lietotājs noņemts!` });
                } else {
                    new Notification({ type: "success", text: `${removedCount} lietotāji noņemti!` });
                }

            }
        });
    });

    document.querySelector("#dim-background").style.opacity = 0.8;
    document.querySelector("#dim-background").classList.toggle("hidden");
    document.querySelector("#remove-user-prompt").classList.toggle("hidden");
});