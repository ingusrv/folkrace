const DEFAULT_OPTIONS = {
    type: "info",
    autoClose: 4,
    canClose: true,
}

export default class Notification {
    #notificationEl;

    constructor(options) {
        console.log("izveidojam notification");

        this.#notificationEl = document.createElement("div");
        this.#notificationEl.classList.add("notification");
        this.#notificationEl.classList.add("show");
        this.update({ ...DEFAULT_OPTIONS, ...options });

        const container = document.querySelector("#notification-container") || this.#createContainer();
        container.append(this.#notificationEl);
    }

    set text(value) {
        console.log("setting notification text", value);
        this.#notificationEl.textContent = value;
    }

    set type(value) {
        console.log("setting notification type", value);
        switch (value) {
            case "info":
                this.#notificationEl.classList.add("info-notification");
                break;
            case "success":
                this.#notificationEl.classList.add("success-notification");
                break;
            case "error":
                this.#notificationEl.classList.add("error-notification");
                break;
            case "warning":
                this.#notificationEl.classList.add("warning-notification");
                break;
            default:
                console.log("Unknown notification type: ", value);
                break;
        }
    }

    set canClose(value) {
        this.#notificationEl.classList.toggle("can-close", value);
        if (value) {
            this.#notificationEl.addEventListener("click", this.remove.bind(this));
        } else {
            this.#notificationEl.removeEventListener("click", this.remove.bind(this));
        }
    }

    set autoClose(value) {
        console.log("settingu up auto close", value)
        setTimeout(this.remove.bind(this), Number(value) * 1000);
    }

    update(options) {
        console.log("updating all options");
        Object.entries(options).forEach(([key, value]) => {
            this[key] = value;
        });
    }

    remove() {
        console.log("removing notification");
        const container = this.#notificationEl.parentElement;
        this.#notificationEl.classList.remove("show");
        this.#notificationEl.addEventListener("animationend", (e) => {
            this.#notificationEl.remove();
            console.log("removed notification element")

            if (container.hasChildNodes()) {
                return;
            }

            container.remove();
            console.log("removed notification container");
        });
    }

    #createContainer() {
        const container = document.createElement("div");
        container.id = "notification-container";
        container.classList.add("notification-container");
        document.body.append(container);
        return container;
    }
}