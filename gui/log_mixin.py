"""Live-Log-Fenster: spiegelt stdout/print() in ein Text-Widget."""


class _StdoutTee:
    """Spiegelt print()/stdout zusätzlich in die Log-Anzeige (siehe LogMixin.on_log_pressed).
    write() wird auch aus Hintergrund-Threads aufgerufen - callback muss daher threadsicher sein
    (siehe LogMixin._on_log_line: schreibt nur in eine deque, Widget-Update läuft über after())."""

    def __init__(self, original, callback):
        self._original = original
        self._callback = callback

    def write(self, text: str) -> int:
        self._original.write(text)
        self._callback(text)
        return len(text)

    def flush(self) -> None:
        self._original.flush()


class LogMixin:
    def _on_log_line(self, text: str) -> None:
        """Wird bei jedem print()/stdout-Write aufgerufen, auch aus Hintergrund-Threads -
        deshalb nur threadsicher in die deque schreiben, Widget-Update über after(0, ...)."""
        self._log_buffer.append(text)
        if self._log_text_widget is not None:
            self.mainwindow.after(0, self._append_log_widget, text)

    def _append_log_widget(self, text: str) -> None:
        widget = self._log_text_widget
        if widget is None or not widget.winfo_exists():
            return
        widget.configure(state="normal")
        widget.insert("end", text)
        widget.see("end")
        widget.configure(state="disabled")

    def on_log_pressed(self):
        """Button 'Log anzeigen': öffnet window_log mit laufend aktualisierter Konsolenausgabe."""
        if getattr(self, 'window_log', None) is None:
            self.window_log = self.builder.get_object('window_log', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_log.configure(background=self._palette()["bg"])
            self._style_widget("label_log_title", "Title.TLabel")
            self._style_widget("button_log_schliessen", "Secondary.TButton")

            text_widget = self.builder.get_object("text_log", self.mainwindow)
            scrollbar = self.builder.get_object("scroll_log", self.mainwindow)
            text_widget.configure(
                yscrollcommand=scrollbar.set,
                wrap="word",
                background="#12161d",
                foreground="#d7dbe3",
                insertbackground="#d7dbe3",
                font=("Courier", 10),
                relief="flat",
            )
            scrollbar.configure(command=text_widget.yview)

            text_widget.insert("end", "".join(self._log_buffer))
            text_widget.see("end")
            text_widget.configure(state="disabled")
            self._log_text_widget = text_widget

            self._make_responsive(self.window_log, self.builder.get_object("frame_log"))
            self._fit_window_to_screen(self.window_log)
        else:
            self.window_log.deiconify()
        self.window_log.lift()
        self.window_log.focus_force()

    def on_log_schliessen_pressed(self):
        """Blendet nur das Log-Fenster aus - _on_log_line schreibt unsichtbar weiter mit."""
        self.window_log.withdraw()
