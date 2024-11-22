# python imports

# 3rd party imports
from fastapi import APIRouter
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.requests import Request


# local imports
from delivery_bridge.webapp.dependencies import (
    NotAuthenticatedException,
    static_dir,
    templates,
)

router = APIRouter()


@router.get("/", response_class=HTMLResponse)
async def get_index(request: Request):
    if request.state.user is None:
        raise NotAuthenticatedException()
    return templates.TemplateResponse("home/home.html", {"request": request})


@router.get("/favicon.ico", response_class=FileResponse)
async def get_favicon():
    return static_dir + "/img/favicon.ico"


@router.get("/test_events", response_class=HTMLResponse)
async def get_test_events(request: Request):
    if request.state.user is None:
        raise NotAuthenticatedException()
    return templates.TemplateResponse("home/event_test.html", {"request": request})


@router.get("/delivery", response_class=HTMLResponse)
async def get_delivery(request: Request):
    if request.state.user is None:
        raise NotAuthenticatedException()
    return templates.TemplateResponse("home/delivery.html", {"request": request})

