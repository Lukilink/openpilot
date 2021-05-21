"""Install exception handler for process crash."""
from selfdrive.swaglog import cloudlog
from selfdrive.version import version

import sentry_sdk
from sentry_sdk.integrations.threading import ThreadingIntegration

def capture_exception(*args, **kwargs):
  cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  try:
    sentry_sdk.capture_exception(*args, **kwargs)
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")

def bind_user(**kwargs):
  sentry_sdk.set_user(kwargs)

def bind_extra(**kwargs):
  for k, v in kwargs.items():
    sentry_sdk.set_tag(k, v)

def init():
  sentry_sdk.init("https://4b660d221577422aba9fcc337813a54a@o374486.ingest.sentry.io/5192596",
                  default_integrations=False, integrations=[ThreadingIntegration(propagate_hub=True)],
                  release=version)
