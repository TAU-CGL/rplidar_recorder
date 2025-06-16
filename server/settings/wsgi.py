"""
WSGI config for settings project.

It exposes the WSGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/5.1/howto/deployment/wsgi/
"""

import os

import django
from django.core.wsgi import get_wsgi_application

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'settings.settings')
def application(environ, start_response):
    os.environ["RPLIDAR_RECORDER_SECRET_KEY"] = environ["RPLIDAR_RECORDER_SECRET_KEY"]
    _application = get_wsgi_application()
    return _application(environ, start_response)