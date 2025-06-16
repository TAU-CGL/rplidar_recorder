"""
WSGI config for settings project.

It exposes the WSGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/5.1/howto/deployment/wsgi/
"""

import os

import django
from django.core.wsgi import WSGIHandler

class WSGIEnvironment(WSGIHandler):
    def __call__(self, environ, start_response):
        os.environ["RPLIDAR_RECORDER_SECRET_KEY"] = environ["RPLIDAR_RECORDER_SECRET_KEY"]
        os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'settings.settings')
        django.setup()
        return super(WSGIEnvironment, self).__call__(environ, start_response)

application = WSGIEnvironment()
