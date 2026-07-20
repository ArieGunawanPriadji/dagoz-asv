# app/models/run.py
from sqlalchemy import Column, Integer, String, Float, Text
from sqlalchemy.dialects.postgresql import TIMESTAMP
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from app.core.database import Base


class MissionRun(Base):
    """
    Satu 'Run' = satu sesi misi kapal.
    Setiap kali kapal mulai misi baru, dibuat satu run baru.
    Semua events dan images dikaitkan ke run ini.
    """
    __tablename__ = "mission_runs"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    run_name = Column(String(100), nullable=False)  # Misal: "Run Latihan 1", "Run Final"
    status = Column(
        String(20), nullable=False, default="PENDING", index=True
    )  # PENDING, RUNNING, FINISHED, ABORTED
    started_at = Column(TIMESTAMP(timezone=True), nullable=True)
    finished_at = Column(TIMESTAMP(timezone=True), nullable=True)
    total_score = Column(Float, nullable=True)  # Diisi oleh scoring engine nanti
    notes = Column(Text, nullable=True)  # Catatan operator
    created_at = Column(TIMESTAMP(timezone=True), server_default=func.now())

    # Relationships: satu run punya banyak events dan images
    events = relationship("MissionEvent", back_populates="run", lazy="selectin")
    images = relationship("MissionImage", back_populates="run", lazy="selectin")
