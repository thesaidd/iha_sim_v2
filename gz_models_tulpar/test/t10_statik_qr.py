#!/usr/bin/env python3
"""Adım 2: qr_irtifa_tarama.py'yi yeniden kullanarak 60 m'de QR modül boyutunu ölçer.
Kullanım: t10_statik_qr.py <cikti_klasoru> [referans|tulpar]   (referans = gz_models_cam_hw/kamera_test)"""
import importlib.util, os, sys
cikti, mod = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "tulpar")
HW = os.path.expanduser("~/iha_sim_v2/gz_models_cam_hw")
sys.argv = ["qr_irtifa_tarama.py", cikti, os.path.join(cikti, "dur")]
spec = importlib.util.spec_from_file_location("qr_irtifa_tarama", os.path.join(HW, "qr_irtifa_tarama.py"))
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
m.ALTITUDES = [40, 60, 80]; m.YAWS = [0, 45]
if mod == "tulpar":
    m.MODEL_SDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "t10_statik_model.sdf")
m.main()
