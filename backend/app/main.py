from contextlib import asynccontextmanager
from fastapi import FastAPI
from app.routers import events, runs, images, dashboard, telemetry, ws
from app.core.database import engine, Base
from app.services.influxdb_service import influx_service
from app.services.telemetry_simulator import simulator

from app.services.mavlink_service import mavlink_service
from app.core.config import settings

# Import semua models agar SQLAlchemy tahu tabel apa saja yang harus dibuat
from app.models import event, run, image  # noqa: F401

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Buat semua tabel yang didefinisikan di Base.metadata
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    
    # Inisialisasi InfluxDB
    await influx_service.connect()
    
    # Mulai service Telemetri otomatis
    if settings.USE_SIMULATOR:
        await simulator.start()
    else:
        await mavlink_service.start()
    
    yield
    
    # Shutdown: Cleanup
    if settings.USE_SIMULATOR:
        await simulator.stop()
    else:
        await mavlink_service.stop()
        
    await influx_service.close()

app = FastAPI(
    title="Dagozilla ASV Backend",
    description="Local-first backend for KKI 2026 ASV Mission Monitoring",
    version="1.0.0",
    lifespan=lifespan
)

# Include routers
app.include_router(events.router)
app.include_router(runs.router)
app.include_router(images.router)
app.include_router(dashboard.router)
app.include_router(telemetry.router)
app.include_router(ws.router)

@app.get("/health")
async def health_check():
    return {"status": "OK", "vessel": "Qing"}