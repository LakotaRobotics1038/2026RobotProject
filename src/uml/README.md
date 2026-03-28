# How to update the UML diagrams

These folders hold **drawing instructions** for our robot code. The computer reads them and shows boxes and lines (a “class diagram”).

There are **two kinds of files** you can change:

1. **One file per Java class**  
   Example: `frc/robot/Robot.mmd` goes with the `Robot` Java class.  
   If you change the real code, update the matching `.mmd` file so the picture stays true.

2. **`relationships.mmd`**  
   This file lists **how classes connect** (arrows like “this uses that”).  
   Change it when you add or remove big connections between types.

---

## The big file you do **not** edit by hand

**`uml.mmd`** is the **full picture** of everything together.  
It is **made automatically**. Do not type inside this file — your changes would disappear the next time someone runs the script.

---

## How to rebuild the big picture

After you edit any small `.mmd` file or `relationships.mmd`, run **one** of these:

### Way A — Gradle (good if you use the project in VS Code)

Open a terminal in the project folder and run:

```text
gradlew mergeUml
```

On Mac or Linux it might be:

```text
./gradlew mergeUml
```

### Way B — PowerShell (Windows)

Open PowerShell in the project folder and run:

```text
.\scripts\merge-uml.ps1
```

When it finishes, **`uml.mmd`** is fresh. Open it in a Mermaid preview, or copy it into [Mermaid Live](https://mermaid.live) to see the drawing.

---

## Quick checklist

1. Edit the **small** diagram file(s) for the class(es) you changed.  
2. Edit **`relationships.mmd`** only if the **arrows** between classes changed.  
3. Run **`gradlew mergeUml`** (or the PowerShell script).  
4. Done — **`uml.mmd`** is updated.

If something looks wrong, ask a mentor and show them which `.mmd` file you changed.
