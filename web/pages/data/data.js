fetch("/api/v1/driveData").then(async (res) => {
    const table = document.querySelector("#run-table");
    const body = await res.json();
    console.log(body);

    body.data.forEach((item, i) => {
        const tr = document.createElement("tr");
        // TODO: fix potential XSS
        // TODO: replace table with message if no data or error
        tr.innerHTML = `
        <td><input class="select-data-checkbox" type="checkbox" name="select-data-checkbox" data-select-data-checkbox data-label="${item.label}" data-elapsed-time="${item.elapsedTime}" data-fps="${item.fps}"></td>
        <td>${++i}</td>
        <td>${item.robotId}</td>
        <td>${item.createdAt}</td>
        <td>${item.algorithm}</td>
        <td>${item.version}</td>
        <td>${item.label}</td>
        <td>${item.fps}</td>
        <td>${item.elapsedTime}</td>
        <td class="short-data">${item.data}</td>
        `;
        table.appendChild(tr);
    });
});