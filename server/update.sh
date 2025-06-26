#!/bin/bash
source secret.sh
git pull
python manage.py makemigrations index
python manage.py migrate
sudo systemctl restart apache2
