#!/usr/bin/env python3

import tkinter as tk
from tkinter import messagebox
import subprocess
import threading
import time
import os
import sys

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

class CarInfotainment:
    def __init__(self):
        self.root = tk.Tk()
        self.setup_window()
        self.setup_gpio()
        self.create_ui()
        self.setup_keybinds()
        
        self.dark_theme = True
        self.apply_theme()
        
    def setup_window(self):
        self.root.title("Car Infotainment")
        self.root.configure(cursor="none")
        
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        
        self.root.geometry(f"{screen_width}x{screen_height}")
        self.root.attributes('-fullscreen', True)
        self.root.attributes('-topmost', True)
        
        self.root.overrideredirect(True)
        
    def setup_gpio(self):
        if not GPIO_AVAILABLE:
            return
            
        try:
            self.VOLUME_UP_PIN = 18
            self.VOLUME_DOWN_PIN = 19
            self.MEDIA_PREV_PIN = 20
            self.MEDIA_NEXT_PIN = 21
            
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.VOLUME_UP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.VOLUME_DOWN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.MEDIA_PREV_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.MEDIA_NEXT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            GPIO.add_event_detect(self.VOLUME_UP_PIN, GPIO.FALLING, 
                                callback=self.volume_up, bouncetime=200)
            GPIO.add_event_detect(self.VOLUME_DOWN_PIN, GPIO.FALLING, 
                                callback=self.volume_down, bouncetime=200)
            GPIO.add_event_detect(self.MEDIA_PREV_PIN, GPIO.FALLING, 
                                callback=self.media_previous, bouncetime=200)
            GPIO.add_event_detect(self.MEDIA_NEXT_PIN, GPIO.FALLING, 
                                callback=self.media_next, bouncetime=200)
        except Exception as e:
            pass
            
    def create_ui(self):
        self.main_frame = tk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        self.title_frame = tk.Frame(self.main_frame)
        self.title_frame.pack(fill=tk.X, pady=(0, 30))
        
        self.title_label = tk.Label(self.title_frame, text="Car Infotainment", 
                                   font=("Arial", 24, "bold"))
        self.title_label.pack(side=tk.LEFT)
        
        self.theme_btn = tk.Button(self.title_frame, text="🌙", font=("Arial", 16),
                                  command=self.toggle_theme, width=3)
        self.theme_btn.pack(side=tk.RIGHT)
        
        self.exit_btn = tk.Button(self.title_frame, text="✕", font=("Arial", 16),
                                 command=self.safe_exit, width=3)
        self.exit_btn.pack(side=tk.RIGHT, padx=(0, 10))
        
        self.grid_frame = tk.Frame(self.main_frame)
        self.grid_frame.pack(expand=True, fill=tk.BOTH)
        
        self.grid_frame.grid_rowconfigure(0, weight=1)
        self.grid_frame.grid_rowconfigure(1, weight=1)
        self.grid_frame.grid_columnconfigure(0, weight=1)
        self.grid_frame.grid_columnconfigure(1, weight=1)
        
        self.create_app_button("🎵\nMusic", self.launch_music, 0, 0)
        self.create_app_button("📱\nPhone", self.launch_phone, 0, 1)
        self.create_app_button("💬\nMessages", self.launch_messages, 1, 0)
        self.create_app_button("🗺️\nMaps", self.launch_maps, 1, 1)
        
        self.status_frame = tk.Frame(self.main_frame)
        self.status_frame.pack(fill=tk.X, pady=(30, 0))
        
        self.status_label = tk.Label(self.status_frame, text="Ready", 
                                    font=("Arial", 12))
        self.status_label.pack(side=tk.LEFT)
        
        self.volume_label = tk.Label(self.status_frame, text="Vol: --", 
                                    font=("Arial", 12))
        self.volume_label.pack(side=tk.RIGHT)
        
        self.update_volume_display()
        
    def create_app_button(self, text, command, row, col):
        btn = tk.Button(self.grid_frame, text=text, command=command,
                       font=("Arial", 18, "bold"), width=12, height=6,
                       relief=tk.RAISED, bd=3)
        btn.grid(row=row, column=col, padx=15, pady=15, sticky="nsew")
        return btn
        
    def setup_keybinds(self):
        self.root.bind('<Escape>', lambda e: self.safe_exit())
        self.root.bind('<F11>', lambda e: self.toggle_fullscreen())
        self.root.bind('<t>', lambda e: self.toggle_theme())
        
    def apply_theme(self):
        if self.dark_theme:
            bg_color = "#2b2b2b"
            fg_color = "#ffffff"
            btn_bg = "#404040"
            btn_fg = "#ffffff"
            theme_icon = "☀️"
        else:
            bg_color = "#f0f0f0"
            fg_color = "#000000"
            btn_bg = "#e0e0e0"
            btn_fg = "#000000"
            theme_icon = "🌙"
            
        self.root.configure(bg=bg_color)
        self.main_frame.configure(bg=bg_color)
        self.title_frame.configure(bg=bg_color)
        self.grid_frame.configure(bg=bg_color)
        self.status_frame.configure(bg=bg_color)
        
        self.title_label.configure(bg=bg_color, fg=fg_color)
        self.status_label.configure(bg=bg_color, fg=fg_color)
        self.volume_label.configure(bg=bg_color, fg=fg_color)
        
        self.theme_btn.configure(text=theme_icon, bg=btn_bg, fg=btn_fg)
        self.exit_btn.configure(bg=btn_bg, fg=btn_fg)
        
        for widget in self.grid_frame.winfo_children():
            if isinstance(widget, tk.Button):
                widget.configure(bg=btn_bg, fg=btn_fg, activebackground=btn_bg)
                
    def toggle_theme(self):
        self.dark_theme = not self.dark_theme
        self.apply_theme()
        
    def update_status(self, message):
        self.status_label.configure(text=message)
        self.root.after(3000, lambda: self.status_label.configure(text="Ready"))
        
    def run_command(self, command, app_name):
        try:
            self.update_status(f"Launching {app_name}...")
            result = subprocess.run(command, shell=True, capture_output=True, 
                                  text=True, timeout=10)
            if result.returncode != 0:
                self.update_status(f"Error: {app_name} not found")
        except subprocess.TimeoutExpired:
            self.update_status(f"Timeout launching {app_name}")
        except Exception as e:
            self.update_status(f"Failed to launch {app_name}")
            
    def launch_music(self):
        players = [
            ("spotify", "Spotify"),
            ("rhythmbox", "Rhythmbox"),
            ("audacious", "Audacious"),
            ("vlc", "VLC"),
            ("mpv --player-operation-mode=pseudo-gui", "MPV")
        ]
        
        for cmd, name in players:
            if self.check_command_exists(cmd.split()[0]):
                threading.Thread(target=self.run_command, 
                               args=(cmd, name), daemon=True).start()
                return
                
        self.update_status("No music player found")
        
    def launch_phone(self):
        bt_managers = [
            ("blueman-manager", "Blueman"),
            ("bluetooth-wizard", "Bluetooth Wizard"),
            ("gnome-bluetooth", "GNOME Bluetooth")
        ]
        
        for cmd, name in bt_managers:
            if self.check_command_exists(cmd):
                threading.Thread(target=self.run_command, 
                               args=(cmd, name), daemon=True).start()
                return
                
        self.update_status("No Bluetooth manager found")
        
    def launch_messages(self):
        self.update_status("Messages: Feature coming soon")
        messagebox.showinfo("Messages", 
                           "Bluetooth SMS sync not yet implemented.\n"
                           "This would connect to your phone for SMS.")
        
    def launch_maps(self):
        nav_apps = [
            ("gnome-maps", "GNOME Maps"),
            ("navit", "Navit"),
            ("osmscout-server", "OSM Scout"),
            ("marble", "Marble"),
            ("google-earth", "Google Earth")
        ]
        
        for cmd, name in nav_apps:
            if self.check_command_exists(cmd):
                threading.Thread(target=self.run_command, 
                               args=(cmd, name), daemon=True).start()
                return
                
        self.update_status("No navigation app found")
        
    def check_command_exists(self, command):
        try:
            subprocess.run(["which", command], check=True, 
                          capture_output=True, text=True)
            return True
        except subprocess.CalledProcessError:
            return False
    
    def volume_up(self, channel=None):
        threading.Thread(target=self._volume_up, daemon=True).start()
        
    def _volume_up(self):
        try:
            subprocess.run(["amixer", "sset", "Master", "5%+"], 
                          capture_output=True)
            self.update_volume_display()
            self.update_status("Volume Up")
        except Exception as e:
            pass
            
    def volume_down(self, channel=None):
        threading.Thread(target=self._volume_down, daemon=True).start()
        
    def _volume_down(self):
        try:
            subprocess.run(["amixer", "sset", "Master", "5%-"], 
                          capture_output=True)
            self.update_volume_display()
            self.update_status("Volume Down")
        except Exception as e:
            pass
            
    def media_previous(self, channel=None):
        threading.Thread(target=self._media_previous, daemon=True).start()
        
    def _media_previous(self):
        try:
            if self.check_command_exists("playerctl"):
                subprocess.run(["playerctl", "previous"], capture_output=True)
                self.update_status("Previous Track")
        except Exception as e:
            pass
            
    def media_next(self, channel=None):
        threading.Thread(target=self._media_next, daemon=True).start()
        
    def _media_next(self):
        try:
            if self.check_command_exists("playerctl"):
                subprocess.run(["playerctl", "next"], capture_output=True)
                self.update_status("Next Track")
        except Exception as e:
            pass
            
    def update_volume_display(self):
        try:
            result = subprocess.run(["amixer", "get", "Master"], 
                                  capture_output=True, text=True)
            import re
            match = re.search(r'\[(\d+)%\]', result.stdout)
            if match:
                volume = match.group(1)
                self.volume_label.configure(text=f"Vol: {volume}%")
            else:
                self.volume_label.configure(text="Vol: --")
        except Exception:
            self.volume_label.configure(text="Vol: --")
            
    def toggle_fullscreen(self):
        current = self.root.attributes('-fullscreen')
        self.root.attributes('-fullscreen', not current)
        
    def safe_exit(self):
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        self.root.quit()
        self.root.destroy()
        
    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.safe_exit()

def main():
    if GPIO_AVAILABLE and os.geteuid() != 0:
        pass
    
    app = CarInfotainment()
    app.run()

if __name__ == "__main__":
    main()