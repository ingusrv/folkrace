fetch("/api/v1/driveData").then(async (res) => {
    const table = document.querySelector("#run-table");
    const body = await res.json();
    console.log(body);

    body.data.forEach((item, i) => {
        const tr = document.createElement("tr");
        // TODO: fix potential XSS
        // TODO: replace table with message if no data or error
        tr.innerHTML = `<td>${++i}</td><td>${item.createdAt}</td><td>${item.elapsedTime}</td><td>${item.data}</td>`;
        table.appendChild(tr);
    });
});