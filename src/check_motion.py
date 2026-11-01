"""Compatibility entry point for current v4 mechanical checks."""
from pathlib import Path
from check_v4_body import main
if __name__=="__main__":main(str(Path(__file__).resolve().parent.parent/"cad/reference/ST3215.step"))
