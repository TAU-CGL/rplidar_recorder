fetch("/api/contraption/list")
    .then(response => {
        return response.json();
    }).then(data => {
        console.log(data);
    });