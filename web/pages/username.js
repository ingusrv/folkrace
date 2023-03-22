fetch("/api/v1/user").then(async (res) => {
    const display = document.querySelector("#username-display");
    const user = await res.json();
    console.log(user);
    display.innerText = user.username;
});