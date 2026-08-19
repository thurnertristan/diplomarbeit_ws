#!/usr/bin/env python3
"""Analytische Abschätzung des Biegemoments und des daraus nötigen
Motor-Drehmoments für das Rohrbiegen - als Grundlage für die Auswahl von
Motor + Getriebe, BEVOR eine Hardware feststeht.

Wichtige Einschränkung: das hier ist eine Handrechnung nach etablierten
Biegeformeln für den Rohrquerschnitt, KEINE FEM-Simulation. Sie
berücksichtigt weder Reibung (Rohr/Dorn/Matrize), noch Kaltverfestigung,
noch Rückfederung - für eine erste Motor-/Getriebe-Dimensionierung mit
Sicherheitsfaktor ist das üblich und ausreichend, für eine exakte
Prozessauslegung nicht. Gazebo (Starrkörperphysik) kann diese Werte NICHT
liefern - eine Rohrverformung ist keine Starrkörper-Bewegung.

Rechenweg:
  1. Elastisches Biegemoment (Fließbeginn) über das elastische
     Widerstandsmoment eines Hohlquerschnitts:
         W = pi/32 * (D_a^4 - D_i^4) / D_a
         M_elastisch = sigma_streck * W
  2. Vollplastisches Biegemoment (Querschnitt komplett durchplastifiziert -
     das ist der praktisch relevante Wert für bleibendes Biegen) über das
     plastische Widerstandsmoment:
         Z = (D_a^3 - D_i^3) / 6
         M_plastisch = sigma_streck * Z
     (Z/W = 4/pi ≈ 1.27 für einen Kreisring - "Formfaktor")
  3. Nötiges Motor-Wellendrehmoment aus dem Biegemoment am Rohr, der
     Getriebeübersetzung und einem Sicherheitsfaktor (deckt Reibung,
     Dynamik, Materialstreuung ab):
         T_motor = M_plastisch * sicherheitsfaktor / (uebersetzung * wirkungsgrad)

Aufruf (Beispiel):
    python3 biegekraft_rechner.py --od 20 --wall 1.5 --material S235 --gear-ratio 50
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

# Fließgrenzen gängiger Werkstoffe (MPa = N/mm^2) - Richtwerte, im Zweifel
# Werkstoff-Datenblatt/Zeugnis prüfen (streut je nach Charge/Wärmebehandlung).
MATERIALS_MPA = {
    "S235": 235,       # Baustahl, unlegiert
    "S355": 355,       # Baustahl, unlegiert (höherfest)
    "1.4301": 215,     # Edelstahl V2A, geglüht
    "1.4571": 220,     # Edelstahl V4A, geglüht
    "AlMg3": 130,      # Aluminium 5754, weich
    "AlMgSi1_T6": 260, # Aluminium 6082-T6
    "Cu-DHP": 60,      # Kupfer, weich
}


@dataclass
class BendingEstimate:
    outer_diameter_mm: float
    wall_thickness_mm: float
    yield_strength_mpa: float
    elastic_section_modulus_mm3: float
    plastic_section_modulus_mm3: float
    elastic_moment_nm: float
    plastic_moment_nm: float

    def motor_shaft_torque_nm(self, gear_ratio: float, efficiency: float = 0.8, safety_factor: float = 1.5) -> float:
        """Nötiges Drehmoment an der Motorwelle, um das vollplastische
        Biegemoment sicher aufzubringen. `efficiency`: Getriebewirkungsgrad
        (0.7-0.9 Schnecke, 0.85-0.95 Planetengetriebe). `safety_factor`
        deckt Reibung/Dynamik/Materialstreuung ab."""
        output_torque = self.plastic_moment_nm * safety_factor
        return output_torque / (gear_ratio * efficiency)


def bending_estimate(outer_diameter_mm: float, wall_thickness_mm: float, yield_strength_mpa: float) -> BendingEstimate:
    """Berechnet elastisches und vollplastisches Biegemoment für ein Rohr
    mit rundem Hohlquerschnitt (siehe Modul-Docstring für die Formeln)."""
    if wall_thickness_mm <= 0 or wall_thickness_mm >= outer_diameter_mm / 2:
        raise ValueError("Wandstärke muss > 0 und kleiner als der Außenradius sein.")

    d_a = outer_diameter_mm
    d_i = outer_diameter_mm - 2 * wall_thickness_mm

    w_elastic = math.pi / 32 * (d_a**4 - d_i**4) / d_a       # mm^3
    z_plastic = (d_a**3 - d_i**3) / 6                         # mm^3

    # sigma [N/mm^2] * W [mm^3] = M [N*mm] -> /1000 für N*m
    m_elastic_nm = yield_strength_mpa * w_elastic / 1000.0
    m_plastic_nm = yield_strength_mpa * z_plastic / 1000.0

    return BendingEstimate(
        outer_diameter_mm=outer_diameter_mm,
        wall_thickness_mm=wall_thickness_mm,
        yield_strength_mpa=yield_strength_mpa,
        elastic_section_modulus_mm3=w_elastic,
        plastic_section_modulus_mm3=z_plastic,
        elastic_moment_nm=m_elastic_nm,
        plastic_moment_nm=m_plastic_nm,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--od", type=float, required=True, help="Rohr-Außendurchmesser in mm")
    parser.add_argument("--wall", type=float, required=True, help="Rohr-Wandstärke in mm")
    material_group = parser.add_mutually_exclusive_group(required=True)
    material_group.add_argument("--material", choices=sorted(MATERIALS_MPA), help="Werkstoff aus der Richtwert-Tabelle")
    material_group.add_argument("--yield-mpa", type=float, help="Fließgrenze direkt in MPa (N/mm^2), falls bekannt")
    parser.add_argument("--gear-ratio", type=float, default=1.0, help="Getriebeübersetzung Motor->Rotationsachse (Standard: 1.0, kein Getriebe)")
    parser.add_argument("--efficiency", type=float, default=0.8, help="Getriebewirkungsgrad, 0-1 (Standard: 0.8)")
    parser.add_argument("--safety-factor", type=float, default=1.5, help="Sicherheitsfaktor für Reibung/Dynamik (Standard: 1.5)")
    args = parser.parse_args()

    yield_mpa = args.yield_mpa if args.yield_mpa is not None else MATERIALS_MPA[args.material]

    estimate = bending_estimate(args.od, args.wall, yield_mpa)
    motor_torque = estimate.motor_shaft_torque_nm(args.gear_ratio, args.efficiency, args.safety_factor)

    print(f"Rohr: Ø{args.od:.1f} mm x {args.wall:.2f} mm Wandstärke, Fließgrenze {yield_mpa:.0f} MPa\n")
    print(f"Elastisches Widerstandsmoment W  = {estimate.elastic_section_modulus_mm3:10.1f} mm^3")
    print(f"Plastisches Widerstandsmoment  Z  = {estimate.plastic_section_modulus_mm3:10.1f} mm^3\n")
    print(f"Biegemoment bei Fließbeginn (elastisch) = {estimate.elastic_moment_nm:8.2f} Nm")
    print(f"Biegemoment vollplastisch (bleibend)    = {estimate.plastic_moment_nm:8.2f} Nm")
    print(
        f"\nNötiges Motor-Wellendrehmoment bei Übersetzung {args.gear_ratio:.1f}:1, "
        f"Wirkungsgrad {args.efficiency:.0%}, Sicherheitsfaktor {args.safety_factor:.1f}:"
    )
    print(f"  T_motor = {motor_torque:.3f} Nm")
    print(
        "\nHinweis: Handrechnung ohne Reibung/Kaltverfestigung/Rückfederung - "
        "als Startpunkt für die Motor-/Getriebeauswahl, nicht als exakte Prozessauslegung."
    )


if __name__ == "__main__":
    main()
