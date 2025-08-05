#!/usr/bin/env python3
import pyray as rl
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.label import gui_text_box

class NagasPilotLayout:
    def __init__(self):
        self.params = Params()
        self.font_regular = gui_app.font(FontWeight.REGULAR)
        self.font_medium = gui_app.font(FontWeight.MEDIUM)
        self.font_bold = gui_app.font(FontWeight.SEMI_BOLD)
        
        # Colors
        self.text_color = rl.WHITE
        self.toggle_on_color = rl.Color(0, 200, 0, 255)
        self.toggle_off_color = rl.Color(100, 100, 100, 255)
        self.button_color = rl.Color(60, 60, 60, 255)
        self.button_hover_color = rl.Color(80, 80, 80, 255)
        
    def render(self, rect: rl.Rectangle):
        # Title
        title_y = rect.y + 20
        gui_text_box(
            rl.Rectangle(rect.x, title_y, rect.width, 80),
            "NagasPilot Settings",
            font_size=60,
            color=self.text_color,
            alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
        )
        
        # Settings sections
        current_y = title_y + 100
        section_height = 60
        
        # DLP Mode
        current_y = self._render_section_header(rect, current_y + 20, "Dynamic Lane Profiling (DLP)")
        current_y = self._render_dlp_mode_selector(rect, current_y + 10)
        
        # Lateral Control
        current_y = self._render_section_header(rect, current_y + 30, "Lateral Control")
        current_y = self._render_toggle(rect, current_y + 10, "np_lat_road_edge_detection", "Road Edge Detection", "Block lane changes when road edge detected")
        current_y = self._render_toggle(rect, current_y + 10, "np_dlp_vision_curve", "Laneless on Curve", "Force laneless mode on curves instead of lanekeep")
        current_y = self._render_toggle(rect, current_y + 10, "np_lat_hand_off_enable", "Hand Off Enable", "Enable hand-off driving mode")
        
        # Longitudinal Control  
        current_y = self._render_section_header(rect, current_y + 30, "Longitudinal Control")
        current_y = self._render_dcp_mode_selector(rect, current_y + 10)
        current_y = self._render_toggle(rect, current_y + 10, "np_energy_optimizer_enabled", "Energy Optimizer", "Enable coasting and brake suppression")
        current_y = self._render_toggle(rect, current_y + 10, "np_curve_speed_enabled", "Curve Speed Control", "Slow down for curves automatically")
        
        # Safety & Monitoring
        current_y = self._render_section_header(rect, current_y + 30, "Safety & Monitoring")
        current_y = self._render_hod_selector(rect, current_y + 10)
        current_y = self._render_ssd_selector(rect, current_y + 10)
        
        # Smart Features
        current_y = self._render_section_header(rect, current_y + 30, "Smart Features")
        current_y = self._render_toggle(rect, current_y + 10, "np_soc_enabled", "Smart Offset Controller", "Automatic lane positioning for safety")
        current_y = self._render_toggle(rect, current_y + 10, "np_yolo_enabled", "Object Detection", "Enhanced object detection with YOLOv8")
        
    def _render_section_header(self, rect: rl.Rectangle, y: float, title: str) -> float:
        gui_text_box(
            rl.Rectangle(rect.x, y, rect.width, 50),
            f"■ {title}",
            font_size=40,
            color=rl.Color(255, 255, 100, 255),  # Yellow header
            alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
        )
        return y + 50
        
    def _render_toggle(self, rect: rl.Rectangle, y: float, param: str, title: str, description: str) -> float:
        # Get current value
        try:
            current_value = self.params.get_bool(param, False)
        except:
            current_value = False
            
        # Toggle button
        toggle_rect = rl.Rectangle(rect.x + rect.width - 100, y + 5, 80, 40)
        toggle_color = self.toggle_on_color if current_value else self.toggle_off_color
        
        # Check for click
        if (rl.is_mouse_button_released(rl.MouseButton.MOUSE_BUTTON_LEFT) and 
            rl.check_collision_point_rec(rl.get_mouse_position(), toggle_rect)):
            try:
                self.params.put_bool(param, not current_value)
            except:
                pass  # Skip if parameter doesn't exist
        
        # Draw toggle
        rl.draw_rectangle_rounded(toggle_rect, 0.5, 10, toggle_color)
        toggle_text = "ON" if current_value else "OFF"
        text_size = rl.measure_text_ex(self.font_medium, toggle_text, 20, 0)
        text_pos = rl.Vector2(
            toggle_rect.x + (toggle_rect.width - text_size.x) / 2,
            toggle_rect.y + (toggle_rect.height - text_size.y) / 2
        )
        rl.draw_text_ex(self.font_medium, toggle_text, text_pos, 20, 0, rl.WHITE)
        
        # Title and description
        gui_text_box(
            rl.Rectangle(rect.x + 20, y, rect.width - 140, 30),
            title,
            font_size=28,
            color=self.text_color,
            alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
        )
        gui_text_box(
            rl.Rectangle(rect.x + 40, y + 25, rect.width - 160, 20),
            description,
            font_size=18,
            color=rl.Color(180, 180, 180, 255),
            alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
        )
        
        return y + 55
        
    def _render_dlp_mode_selector(self, rect: rl.Rectangle, y: float) -> float:
        try:
            current_mode = int(self.params.get("np_dlp_mode", "0"))
        except:
            current_mode = 0
            
        modes = ["Off", "Lanekeep", "Laneless", "DLP"]
        button_width = 120
        button_spacing = 10
        
        gui_text_box(
            rl.Rectangle(rect.x + 20, y, 200, 30),
            "DLP Mode:",
            font_size=28,
            color=self.text_color,
            alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
        )
        
        for i, mode in enumerate(modes):
            button_x = rect.x + 20 + i * (button_width + button_spacing)
            button_rect = rl.Rectangle(button_x, y + 35, button_width, 40)
            
            is_selected = (i == current_mode)
            is_hovered = rl.check_collision_point_rec(rl.get_mouse_position(), button_rect)
            
            # Handle click
            if (rl.is_mouse_button_released(rl.MouseButton.MOUSE_BUTTON_LEFT) and is_hovered):
                try:
                    self.params.put("np_dlp_mode", str(i))
                except:
                    pass
            
            # Draw button
            button_color = self.toggle_on_color if is_selected else (self.button_hover_color if is_hovered else self.button_color)
            rl.draw_rectangle_rounded(button_rect, 0.1, 5, button_color)
            
            # Draw text
            text_size = rl.measure_text_ex(self.font_medium, mode, 22, 0)
            text_pos = rl.Vector2(
                button_rect.x + (button_rect.width - text_size.x) / 2,
                button_rect.y + (button_rect.height - text_size.y) / 2
            )
            rl.draw_text_ex(self.font_medium, mode, text_pos, 22, 0, rl.WHITE)
            
        return y + 85
        
    def _render_dcp_mode_selector(self, rect: rl.Rectangle, y: float) -> float:
        try:
            current_mode = int(self.params.get("np_dcp_mode", "0"))
        except:
            current_mode = 0
            
        modes = ["Off", "Highway", "Urban", "DCP"]
        button_width = 120
        button_spacing = 10
        
        gui_text_box(
            rl.Rectangle(rect.x + 20, y, 200, 30),
            "DCP Mode:",
            font_size=28,
            color=self.text_color,
            alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
        )
        
        for i, mode in enumerate(modes):
            button_x = rect.x + 20 + i * (button_width + button_spacing)
            button_rect = rl.Rectangle(button_x, y + 35, button_width, 40)
            
            is_selected = (i == current_mode)
            is_hovered = rl.check_collision_point_rec(rl.get_mouse_position(), button_rect)
            
            # Handle click
            if (rl.is_mouse_button_released(rl.MouseButton.MOUSE_BUTTON_LEFT) and is_hovered):
                try:
                    self.params.put("np_dcp_mode", str(i))
                except:
                    pass
            
            # Draw button
            button_color = self.toggle_on_color if is_selected else (self.button_hover_color if is_hovered else self.button_color)
            rl.draw_rectangle_rounded(button_rect, 0.1, 5, button_color)
            
            # Draw text
            text_size = rl.measure_text_ex(self.font_medium, mode, 22, 0)
            text_pos = rl.Vector2(
                button_rect.x + (button_rect.width - text_size.x) / 2,
                button_rect.y + (button_rect.height - text_size.y) / 2
            )
            rl.draw_text_ex(self.font_medium, mode, text_pos, 22, 0, rl.WHITE)
            
        return y + 85
        
    def _render_hod_selector(self, rect: rl.Rectangle, y: float) -> float:
        try:
            current_level = int(self.params.get("np_hod_duration_level", "0"))
        except:
            current_level = 0
            
        levels = ["2min", "5min", "10min", "Forever"]
        return self._render_level_selector(rect, y, "Hand Off Duration:", "np_hod_duration_level", levels, current_level)
        
    def _render_ssd_selector(self, rect: rl.Rectangle, y: float) -> float:
        try:
            current_level = int(self.params.get("np_ssd_duration_level", "0"))
        except:
            current_level = 0
            
        levels = ["2min", "5min", "10min", "Forever"]
        return self._render_level_selector(rect, y, "Stand Still Duration:", "np_ssd_duration_level", levels, current_level)
        
    def _render_level_selector(self, rect: rl.Rectangle, y: float, title: str, param: str, levels: list, current_level: int) -> float:
        button_width = 100
        button_spacing = 10
        
        gui_text_box(
            rl.Rectangle(rect.x + 20, y, 300, 30),
            title,
            font_size=28,
            color=self.text_color,
            alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
        )
        
        for i, level in enumerate(levels):
            button_x = rect.x + 20 + i * (button_width + button_spacing)
            button_rect = rl.Rectangle(button_x, y + 35, button_width, 40)
            
            is_selected = (i == current_level)
            is_hovered = rl.check_collision_point_rec(rl.get_mouse_position(), button_rect)
            
            # Handle click
            if (rl.is_mouse_button_released(rl.MouseButton.MOUSE_BUTTON_LEFT) and is_hovered):
                try:
                    self.params.put(param, str(i))
                except:
                    pass
            
            # Draw button
            button_color = self.toggle_on_color if is_selected else (self.button_hover_color if is_hovered else self.button_color)
            rl.draw_rectangle_rounded(button_rect, 0.1, 5, button_color)
            
            # Draw text
            text_size = rl.measure_text_ex(self.font_medium, level, 20, 0)
            text_pos = rl.Vector2(
                button_rect.x + (button_rect.width - text_size.x) / 2,
                button_rect.y + (button_rect.height - text_size.y) / 2
            )
            rl.draw_text_ex(self.font_medium, level, text_pos, 20, 0, rl.WHITE)
            
        return y + 85