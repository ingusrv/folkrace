import requests

url = "http://localhost:3000/api/v1/driveData"
data = {
    "elapsedTime": 10,
    "data": [[0, 1, 0], [0, 1, 0], [1, 1, 0], [0, 1, 1], [0, 1, 0]]
}
res = requests.post(url, json=data)
print(res.text)
