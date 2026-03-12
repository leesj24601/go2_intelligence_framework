from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional

try:
    import numpy as np
except Exception as exc:
    np = None
    _numpy_import_error = str(exc)
else:
    _numpy_import_error = ""

try:
    import sounddevice as sd
except Exception as exc:
    sd = None
    _sounddevice_import_error = str(exc)
else:
    _sounddevice_import_error = ""

try:
    from faster_whisper import WhisperModel
except Exception as exc:
    WhisperModel = None
    _faster_whisper_import_error = str(exc)
else:
    _faster_whisper_import_error = ""


@dataclass
class VoiceRecognitionResult:
    text: str = ""
    error: str = ""


class VoiceCommandListener:
    SAMPLE_RATE = 16000
    CHANNELS = 1
    ENERGY_THRESHOLD = 0.008

    def __init__(self) -> None:
        self._model: Optional[WhisperModel] = None
        self._model_name = os.getenv("GO2_STT_MODEL", "base")
        self._device = os.getenv("GO2_STT_DEVICE", "auto")
        self._compute_type = os.getenv("GO2_STT_COMPUTE_TYPE", "default")
        self._download_root = os.getenv("GO2_STT_DOWNLOAD_ROOT") or None

    @property
    def available(self) -> bool:
        return not self.unavailable_reason

    @property
    def unavailable_reason(self) -> str:
        if WhisperModel is None:
            return f"faster-whisper unavailable ({_faster_whisper_import_error})"
        if np is None:
            return f"numpy unavailable ({_numpy_import_error})"
        if sd is None:
            return f"sounddevice unavailable ({_sounddevice_import_error})"
        return ""

    def listen_once(
        self,
        timeout_sec: float = 3.0,
        phrase_time_limit_sec: float = 4.0,
        language: str = "ko-KR",
    ) -> VoiceRecognitionResult:
        if not self.available:
            return VoiceRecognitionResult(error=self.unavailable_reason)

        record_seconds = max(timeout_sec + phrase_time_limit_sec, 1.0)
        try:
            audio = sd.rec(
                int(record_seconds * self.SAMPLE_RATE),
                samplerate=self.SAMPLE_RATE,
                channels=self.CHANNELS,
                dtype="float32",
            )
            sd.wait()
        except Exception as exc:
            return VoiceRecognitionResult(error=f"microphone record failed: {exc}")

        waveform = audio.reshape(-1) if audio.ndim > 1 else audio
        peak = float(np.max(np.abs(waveform))) if waveform.size else 0.0
        if peak < self.ENERGY_THRESHOLD:
            return VoiceRecognitionResult(error="no speech detected")

        try:
            model = self._get_model()
            segments, _ = model.transcribe(
                waveform,
                language=self._normalize_language(language),
                task="transcribe",
                vad_filter=True,
                without_timestamps=True,
            )
            text = " ".join(segment.text.strip() for segment in segments if segment.text.strip()).strip()
        except Exception as exc:
            return VoiceRecognitionResult(error=f"transcription failed: {exc}")

        if not text:
            return VoiceRecognitionResult(error="empty transcript")
        return VoiceRecognitionResult(text=text)

    def _get_model(self) -> WhisperModel:
        if self._model is None:
            self._model = WhisperModel(
                self._model_name,
                device=self._device,
                compute_type=self._compute_type,
                download_root=self._download_root,
            )
        return self._model

    def _normalize_language(self, language: str) -> str:
        normalized = language.strip().lower()
        if not normalized:
            return "ko"
        return normalized.split("-", 1)[0]
