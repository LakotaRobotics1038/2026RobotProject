# 2026RobotProject — full UML (all members)

Member notation uses **UML-style** spacing in Mermaid: `visibility name type` for attributes and `name(params) returnType` for operations (no ` : ` separator, so the diagram does not show doubled colons). Mermaid static members use a `$` suffix on the name. Package-private Java members use `~`.

Association lines at the bottom of the diagram are **simplified**: command→subsystem and joystick→subsystem links are omitted because they are already listed as fields on each class; inheritance, inner types, and Robot-level wiring are kept.

**Edit per-class files** under [`src/uml/frc/...`](src/uml/frc/robot/) and [`src/uml/edu/...`](src/uml/edu/wpi/first/wpilibj/TimedRobot.mmd), and [`src/uml/relationships.mmd`](src/uml/relationships.mmd) for associations. Then regenerate the combined diagram:

`.\gradlew mergeUml` or `.\scripts\merge-uml.ps1`

The assembled file [`src/uml/uml.mmd`](src/uml/uml.mmd) is **auto-generated** (do not edit it by hand). Open that file in an editor with Mermaid preview, or paste its contents into the [Mermaid Live Editor](https://mermaid.live), to view the full diagram.
