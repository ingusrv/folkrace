let dataToCompare = [];
let checkboxes = undefined;

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

    checkboxes = document.querySelectorAll("[data-select-data-checkbox]");
    loadCheckboxes(checkboxes);
});

function loadCheckboxes(checkboxes) {
    checkboxes.forEach((checkbox) => {
        checkbox.addEventListener("click", (e) => {
            const label = e.target.dataset.label;
            const fps = e.target.dataset.fps;
            const data = { label: label, fps: fps };
            console.log(data);

            for (item of dataToCompare) {
                if (item.label == data.label && item.fps == data.fps) {
                    console.log("dati sakrīt");
                    const index = dataToCompare.indexOf(item);
                    console.log("index", index);
                    dataToCompare.splice(index, 1);
                    console.log("data", dataToCompare);
                    return;
                }
            }
            console.log("dati nesakrīt");
            dataToCompare.push(data);
            console.log("data", dataToCompare);
        });
    });
}

const compareButton = document.querySelector("#compare-data");
const dialog = document.querySelector("#chart-dialog");
const ctx = document.getElementById('compare-chart');

const closeDialog = document.querySelector("#close-dialog");

compareButton.addEventListener("click", (e) => {
    let labels = []
    let data = []
    for (item of dataToCompare) {
        labels.push(item.label);
        data.push(item.fps);
    }

    dialog.showModal();

    let chart = new Chart(ctx, {
        type: 'bar',
        data: {
            labels: labels,
            datasets: [{
                label: 'Vidējais FPS',
                data: data,
                borderWidth: 1
            }]
        },
        options: {
            scales: {
                y: {
                    beginAtZero: true
                }
            }
        }
    });

    closeDialog.addEventListener("submit", (e) => {
        console.log("dialog closed");
        chart.destroy();
    })
});