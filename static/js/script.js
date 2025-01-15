document.addEventListener("DOMContentLoaded", () => {
    const mascot = document.getElementById("mascot");

    // Simulate real-time location updates from the Flask API
    function updateLocation() {
        fetch('/location')
            .then(response => response.json())
            .then(data => {
                const { x, y } = data;
                mascot.style.transform = `translate(${x}px, ${y}px)`;
            });
    }

    setInterval(updateLocation, 1000); // Update every second
});
