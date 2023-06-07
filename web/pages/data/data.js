import Notification from "../notification.js";

const dataTable = document.querySelector("#data-table").querySelector("tbody");

function getData() {
    dataTable.replaceChildren();
    let dataToCompare = [];
    fetch("/api/driveData").then(async (res) => {
        if (res.status >= 400) {
            new Notification({ type: "error", text: "Notika kļūda ielādējot braucienu datus!" });
            return;
        }

        const body = await res.json();
        console.log(body);

        if (Object.entries(body.data).length === 0) {
            dataTable.textContent = "Nav braucienu datu";
            return;
        }

        body.data.forEach((data, i) => {
            const row = document.createElement("tr");

            // dati
            const checkbox = document.createElement("input");
            checkbox.type = "checkbox";
            const number = document.createElement("td");
            number.textContent = ++i;
            const robotId = document.createElement("td");
            robotId.textContent = data.robotId;
            const createdAt = document.createElement("td");
            createdAt.textContent = data.createdAt;
            const algorithm = document.createElement("td");
            algorithm.textContent = data.algorithm;
            const version = document.createElement("td");
            version.textContent = data.version;
            const averageFps = document.createElement("td");
            averageFps.textContent = data.averageFps;
            const elapsedTime = document.createElement("td");
            elapsedTime.textContent = data.elapsedTime;
            const showData = document.createElement("button");
            showData.classList.add("button", "secondary", "on-secondary");
            showData.textContent = "Skatīt";

            // pogu funkcionalitāte
            checkbox.addEventListener("click", () => {
                const dataToSave = {
                    note: `robots "${data.robotId}" @ ${data.createdAt}`,
                    averageFps: data.averageFps
                };
                console.log(data);

                for (let item of dataToCompare) {
                    if (item.note == dataToSave.note && item.averageFps == dataToSave.averageFps) {
                        console.log("dati sakrīt");
                        const index = dataToCompare.indexOf(item);
                        console.log("index", index);
                        dataToCompare.splice(index, 1);
                        console.log("data", dataToCompare);
                        return;
                    }
                }
                console.log("dati nesakrīt");
                dataToCompare.push(dataToSave);
                console.log("data", dataToCompare);
            });
            showData.addEventListener("click", () => {
                const dataDialog = document.querySelector("#view-data-dialog");
                const dataDialogText = document.querySelector("#text-window");

                dataDialog.showModal();
                dataDialogText.textContent = data.data;
            });

            row.append(checkbox, number, robotId, createdAt, algorithm, version, averageFps, elapsedTime, showData);
            dataTable.appendChild(row);
        });
    });

    const compareData = document.querySelector("#compare-data");
    compareData.addEventListener("click", (e) => {
        const dialog = document.querySelector("#chart-dialog");
        const ctx = document.querySelector("#comparison-chart");
        const closeDialog = document.querySelector("#close-chart-dialog");

        let labels = [];
        let data = [];
        for (let item of dataToCompare) {
            labels.push(item.note);
            data.push(item.averageFps);
        }

        dialog.showModal();

        Chart.defaults.borderColor = "#847e89";
        Chart.defaults.color = "#f5f5f5";
        const chart = new Chart(ctx, {
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
            chart.destroy();
        });
    });
}

getData();