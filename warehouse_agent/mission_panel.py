"""Read-only Qt mission display. All content comes from the supervisor snapshot."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QListWidget, QVBoxLayout, QWidget

from .visual_state import mission_label, parcel_color, parcel_label, stop_label


class MissionPanel(QWidget):
    def __init__(self, world):
        super().__init__()
        self.setWindowTitle("Warehouse · Mission view")
        self.resize(580, 720)
        self.setStyleSheet("""
            QWidget { background: #f3f5f8; color: #17273d; font: 14px 'DejaVu Sans'; }
            QLabel#title { font-size: 25px; font-weight: bold; }
            QLabel#caption { color: #64748b; font-size: 12px; }
            QLabel#badge { color: #155e75; background: #dff4f7; padding: 9px 12px; border-radius: 7px; font-weight: bold; }
            QLabel#goal { background: white; border: 1px solid #dce2ea; border-radius: 9px; padding: 17px; font-size: 19px; font-weight: bold; }
            QFrame#parcel { background: white; border: 1px solid #dce2ea; border-radius: 9px; }
            QFrame#parcel QLabel { background: transparent; border: none; }
            QListWidget { background: white; border: 1px solid #dce2ea; border-radius: 9px; padding: 8px; }
            QListWidget::item { padding: 7px; }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 20)
        layout.setSpacing(14)
        self.title = QLabel("Warehouse mission")
        self.title.setObjectName("title")
        layout.addWidget(self.title)
        self.connection = QLabel("Waiting for the agent")
        self.connection.setObjectName("caption")
        layout.addWidget(self.connection)
        self.badge = QLabel("Ready")
        self.badge.setObjectName("badge")
        layout.addWidget(self.badge)
        layout.addWidget(QLabel("CURRENT GOAL"))
        self.goal = QLabel("No active goal")
        self.goal.setObjectName("goal")
        self.goal.setWordWrap(True)
        layout.addWidget(self.goal)
        self.cards = {}
        layout.addWidget(QLabel("PARCELS"))
        for pid in world.config["parcels"]:
            card = QFrame()
            card.setObjectName("parcel")
            row = QHBoxLayout(card)
            row.setContentsMargins(14, 12, 14, 12)
            identity = QLabel(pid)
            identity.setStyleSheet("font-size: 21px; font-weight: bold;")
            row.addWidget(identity)
            row.addStretch()
            state = QLabel()
            state.setMinimumWidth(240)
            state.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row.addWidget(state)
            layout.addWidget(card)
            self.cards[pid] = (identity, state)
        self.remaining_title = QLabel("REMAINING STOPS")
        layout.addWidget(self.remaining_title)
        self.remaining = QListWidget()
        self.remaining.setFocusPolicy(Qt.NoFocus)
        self.remaining.setSelectionMode(QListWidget.NoSelection)
        layout.addWidget(self.remaining, 1)
        self.footer = QLabel("Amber: waiting   ·   Blue: onboard   ·   Green: delivered")
        self.footer.setObjectName("caption")
        self.footer.setWordWrap(True)
        layout.addWidget(self.footer)

    def update_state(self, state, connection, gazebo):
        marker_status = "" if gazebo is None else ("  ·  Gazebo markers connected" if gazebo else "  ·  Waiting for Gazebo markers")
        self.connection.setText(connection + marker_status)
        self.badge.setText(mission_label(state))
        self.goal.setText(stop_label(state["active"]) if state["active"] else "No active goal")
        for pid, (identity, label) in self.cards.items():
            p = state["parcels"][pid]
            color = parcel_color(p)
            rgb = ",".join(str(round(v*255)) for v in color[:3])
            identity.setStyleSheet(f"font-size: 21px; font-weight: bold; color: rgb({rgb});")
            place = "On robot" if p["state"] == "onboard" else p["drop"] if p["state"] == "delivered" else p["pickup"]
            label.setText(parcel_label(p)+"\n"+place.replace("_", " "))
        stops = state.get("remaining_stops", [])
        self.remaining_title.setText(f"REMAINING STOPS · {len(stops)}" if not state.get("plan_pending") else "REMAINING STOPS")
        lines = [f"{i+1:02d}   {stop_label(s)}" for i, s in enumerate(stops)]
        if state.get("plan_pending"):
            lines = ["Order will refresh at the next action boundary."]
        elif not lines:
            lines = ["No stops queued."]
        if [self.remaining.item(i).text() for i in range(self.remaining.count())] != lines:
            self.remaining.clear()
            self.remaining.addItems(lines)
