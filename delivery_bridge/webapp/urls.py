# 3rd party imports
from fastapi import APIRouter

# local imports
from delivery_bridge.webapp.apps.home.urls import router as home_router
from delivery_bridge.webapp.apps.users.urls import router as user_routers
from delivery_bridge.webapp.apps.waypoints.urls import router as waypoints_router
from delivery_bridge.webapp.apps.ros2_app.urls import router as ros2_router
from delivery_bridge.webapp.apps.navigation.urls import router as navigation_router

router = APIRouter()

router.include_router(home_router, prefix="")
router.include_router(user_routers, prefix="/users", tags=["Usuarios"]) # include_in_schema=False
router.include_router(waypoints_router, prefix="/waypoints")
router.include_router(ros2_router, prefix="/ros", tags=["ROS2"])
router.include_router(navigation_router, prefix="/navigation", tags=["Navigation"])
