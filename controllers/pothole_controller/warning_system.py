"""
warning_system.py - Multi-modal hazard signaling subsystem.
Controls Red/Green LEDs, Speaker (playing sounds/alarm.wav), and rear LCD Display (Webots Display API).
Mirrors all visual/audio status updates to console.
"""

import os
from typing import Optional
from config import LCD_WIDTH, LCD_HEIGHT


class WarningSystem:
    def __init__(
        self,
        led_red: Optional[any] = None,
        led_green: Optional[any] = None,
        speaker: Optional[any] = None,
        display: Optional[any] = None,
        sound_path: Optional[str] = None,
    ):
        self.led_red = led_red
        self.led_green = led_green
        self.speaker = speaker
        self.display = display
        self.last_lcd_text = ""

        # Locate alarm.wav sound file
        if sound_path is None:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            self.sound_path = os.path.join(base_dir, "sounds", "alarm.wav")
        else:
            self.sound_path = sound_path

        # Initialize LCD background if present
        if self.display is not None:
            try:
                self.display.setFont("Arial", 14, True)
            except Exception:
                pass

    def show_road_clear(self):
        """Activates green LED, clears red LED, and renders ROAD CLEAR on LCD."""
        if self.led_red:
            self.led_red.set(0)
        if self.led_green:
            self.led_green.set(1)

        self._render_lcd(
            bg_color=0x0F381E,     # Deep forest green
            header_color=0x2ECC71, # Bright emerald
            header_text="ROAD CLEAR",
            sub_text="Cruising safely",
        )

    def show_pothole_alert(self, pothole_id: str = "POTHOLE", distance_m: float = 0.0):
        """Activates red LED, turns off green LED, sounds alarm, and renders alert banner."""
        if self.led_green:
            self.led_green.set(0)
        if self.led_red:
            self.led_red.set(1)

        # Trigger acoustic buzzer via Webots Speaker node with console fallback
        self.play_buzzer()

        self._render_lcd(
            bg_color=0x5C0B0B,     # Deep crimson
            header_color=0xFF3B30, # Vivid red
            header_text="POTHOLE DETECTED",
            sub_text=f"ROBOT STOPPED ({pothole_id})",
        )

    def show_detection_off(self, blind_mode: bool = False):
        """Fail-safe display: both LEDs off."""
        if self.led_red:
            self.led_red.set(0)
        if self.led_green:
            self.led_green.set(0)

        if blind_mode:
            self._render_lcd(
                bg_color=0x3A3A3C,     # Dark charcoal
                header_color=0xFF9F0A, # Caution amber
                header_text="NOT MONITORING",
                sub_text="Fail-safe override active",
            )
        else:
            self._render_lcd(
                bg_color=0x1C1C1E,     # Near black
                header_color=0x8E8E93, # Neutral grey
                header_text="DETECTION OFF",
                sub_text="Vehicle immobilized",
            )

    def show_finished(self):
        """Goal state at end of road."""
        if self.led_red:
            self.led_red.set(0)
        if self.led_green:
            self.led_green.set(1)

        self._render_lcd(
            bg_color=0x0A2540,     # Deep navy
            header_color=0x64D2FF, # Cyan
            header_text="FINISH REACHED",
            sub_text="Road section completed",
        )

    def play_buzzer(self):
        """Plays alarm.wav via Webots Speaker node, falling back to console message."""
        sound_played = False
        if self.speaker is not None and os.path.exists(self.sound_path):
            try:
                # Webots Speaker.playSound(speaker, file, volume, pitch, balance, loop)
                self.speaker.playSound(
                    self.speaker,
                    self.sound_path,
                    1.0,  # volume
                    1.0,  # pitch
                    0.0,  # balance
                    False # loop
                )
                sound_played = True
            except Exception as e:
                # Fallback on mock or API differences
                try:
                    self.speaker.playSound(self.sound_path, 1.0)
                    sound_played = True
                except Exception:
                    pass

        if not sound_played:
            print("[BUZZER SOUND] >>> *BEEP-BEEP* Acoustic warning buzzer activated <<<")

    def _render_lcd(self, bg_color: int, header_color: int, header_text: str, sub_text: str):
        """Renders stylized message on the Webots Display and mirrors to console on change."""
        full_text = f"{header_text} | {sub_text}"
        if full_text != self.last_lcd_text:
            print(f"[LCD DISPLAY] [{header_text}] - {sub_text}")
            self.last_lcd_text = full_text

        if self.display is not None:
            try:
                # 1. Fill LCD background canvas (256x128)
                self.display.setColor(bg_color)
                self.display.fillRectangle(0, 0, LCD_WIDTH, LCD_HEIGHT)

                # 2. Draw border frame
                self.display.setColor(header_color)
                self.display.fillRectangle(0, 0, LCD_WIDTH, 4)
                self.display.fillRectangle(0, LCD_HEIGHT - 4, LCD_WIDTH, 4)
                self.display.fillRectangle(0, 0, 4, LCD_HEIGHT)
                self.display.fillRectangle(LCD_WIDTH - 4, 0, 4, LCD_HEIGHT)

                # 3. Draw header text
                self.display.setColor(header_color)
                self.display.drawText(header_text, 16, 32)

                # 4. Draw sub text
                self.display.setColor(0xFFFFFF)
                self.display.drawText(sub_text, 16, 72)
            except Exception:
                pass
