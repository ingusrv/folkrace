const form = document.querySelector("form");
form.addEventListener("submit", (e) => {
    console.log("logging in...");
    e.preventDefault();
    const user = {
        username: form.querySelector("#username").value,
        password: form.querySelector("#password").value
    };

    fetch("/login/", {
        method: "POST",
        mode: "same-origin",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify(user)
    }).then(async (res) => {
        const body = await res.json();
        console.log(body);
        const statusMessage = document.querySelector("#status-message");
        if (res.status >= 400) {
            statusMessage.style.color = "lightcoral";
            statusMessage.innerText = body.message;
        }
        if (res.status === 200) {
            statusMessage.style.color = "lightgreen";
            statusMessage.innerText = body.message;
            window.location.href = "/data/";
        }
    });
});