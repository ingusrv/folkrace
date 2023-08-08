import Notification from "./notification.js";

const form = document.querySelector("form");
form.addEventListener("submit", (e) => {
    console.log("logging in...");
    e.preventDefault();
    const user = {
        username: form.querySelector("#username").value,
        password: form.querySelector("#password").value
    };

    fetch("/api/auth/login", {
        method: "POST",
        mode: "same-origin",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify(user)
    }).then(async (res) => {
        const body = await res.json();
        console.log(body);
        if (res.status === 200) {
            new Notification({ type: "success", text: body.message });
            window.location.href = "/data";
            return;
        }

        new Notification({ type: "error", text: body.message });
    });
});