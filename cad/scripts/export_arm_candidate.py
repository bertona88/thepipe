"""Export opt-in distal geometry from the executed machine configuration."""
from pathlib import Path
import argparse
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipe_cad.arm_candidate import export_candidate

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('configuration', type=Path)
parser.add_argument('--output', type=Path, default=Path('build/cad/arm_candidate'))
parser.add_argument('--opening-m', type=float)
args = parser.parse_args()
export_candidate(args.configuration, args.output, args.opening_m)
