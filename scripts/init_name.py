import os
import json
import requests


SERVER_URL = os.environ["SERVER_URL"]
UUID_FILE = "/home/ubuntu/.contraption_uuid"

def main():
    if os.path.exists(UUID_FILE):
        return
    url = f"{SERVER_URL}/api/contraption/register"
    response = requests.get(url)
    response = json.loads(response.content)
    uuid = response["uuid"]
    with open(UUID_FILE, "w") as f:
        f.write(uuid)

if __name__ == "__main__":
    main()