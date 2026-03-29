# Generated UML Diagrams

This directory contains automatically generated UML class diagrams for the FRC robot project.

## How it works

UML diagrams are generated using [UMLDoclet](https://github.com/talsma-ict/umldoclet), a Javadoc doclet that produces UML class diagrams embedded in Javadoc HTML documentation.

## Generation

- **Automatic**: A GitHub Actions workflow runs on every push to `main` and commits updated diagrams
- **Manual**: Run locally with `./gradlew generateUml`

## Viewing

- Open `index.html` in a browser to view the full Javadoc with embedded UML diagrams
- SVG diagrams can be viewed directly in GitHub or any browser
- Each class has its own UML diagram showing relationships and members

## Note

These files are auto-generated. Manual edits will be overwritten on the next generation.
For manually curated diagrams, see `src/uml/`.
