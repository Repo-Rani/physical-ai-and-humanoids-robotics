---
id: 0003
title: Textbook Implementation Blueprint
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5
feature: 001-book-master-plan
branch: master
user: Repo-Rani
command: /sp.plan
labels: ["implementation-plan", "docusaurus", "physical-ai", "robotics", "textbook", "algolia", "quiz-platform", "modular-phases", "ci-cd", "constitution-check"]
links:
  spec: specs/001-book-master-plan/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-book-master-plan/plan.md
tests:
  - none
---

## Prompt

```yaml
---
id: 0003
title: Textbook Implementation Blueprint
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5
feature: 001-book-master-plan
branch: master
user: Repo-Rani
command: /sp.plan
labels: ["implementation-plan", "docusaurus", "physical-ai", "robotics", "textbook"]
links:
  spec: specs/001-book-master-plan/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-book-master-plan/plan.md
tests:
  - none
---

## Prompt

You are an expert software architect designing an implementation plan for the Physical AI & Humanoid Robotics Textbook platform.

**Context:**
- Project: Docusaurus-based educational textbook (5 modules, 30 chapters, 13 weeks)
- Spec: `specs/001-book-master-plan/spec.md` (with clarifications from session 2025-12-09)
- Constitution: `.specify/memory/constitution.md` (v3.0.0, 6 principles)
- Clarifications Applied:
  1. Assessment checkpoints → External quiz platforms (Quizlet/Google Forms) via iframe embeds
  2. Search solution → Algolia DocSearch (free for open-source)
  3. Homepage content → Course overview with module cards

**Your Task:**
Generate a comprehensive implementation plan (`specs/001-book-master-plan/plan.md`) following the plan-template structure with enhanced modular phases.

**Requirements:**

1. **Technical Context**
   - Language/version: Docusaurus 3.0+, React 18+, Python 3.10+ (code examples)
   - Primary dependencies: Algolia DocSearch, Mermaid.js, ROS 2 Humble, Docker
   - Storage, testing strategy, target platform, performance goals, constraints

2. **Constitution Check**
   - Validate alignment with all 6 principles from constitution.md
   - Explicitly state PASS/FAIL for each principle

3. **Project Structure**
   - Source code organization (docs/, static/, src/components/, contracts/)
   - Build artifacts (build/, .docusaurus/)
   - Configuration files (docusaurus.config.js, sidebars.js, package.json)

4. **Phase 0: Research**
   - 8 research tasks (R0.1 through R0.8) covering:
     - Docusaurus + React integration patterns
     - Algolia DocSearch application process
     - Quiz platform embed best practices
     - Homepage module card design patterns
     - Frontmatter validation (JSON Schema)
     - Mermaid diagram accessibility
     - Broken link checker integration
     - Code example download buttons

5. **Phase 0.5: Prototyping/Spikes (Optional)**
   - High-risk validations:
     - Algolia search prototype
     - Quiz embed test (iframe + responsive)
     - Module cards component
     - Frontmatter validation script

6. **Phase 1: Design Artifacts**
   - data-model.md (document 9 entities from spec)
   - contracts/ directory with:
     - chapter-frontmatter.schema.json (JSON Schema for YAML validation)
     - sidebar-structure.ts (TypeScript interface)
     - module-card.interface.ts (TypeScript interface)
     - hardware-config.schema.json (JSON Schema)
   - quickstart.md (developer onboarding guide)

7. **Phase 2: Implementation (10 modular phases)**
   - 2.0: Foundational Setup (Docusaurus init, dependencies, basic config)
   - 2.1: Homepage & Navigation (User Story 1 - P1)
   - 2.2: Chapter Template (User Story 2 - P1)
   - 2.3: Code Examples (User Story 3 - P1)
   - 2.4: Diagrams (User Story 4 - P2)
   - 2.5: Exercises (User Story 5 - P2)
   - 2.6: Hardware Guide (User Story 6 - P2)
   - 2.7: Search Integration (User Story 7 - P3)
   - 2.8: Assessment Checkpoints (User Story 8 - P3)
   - 2.9: Glossary & Resources
   - 2.10: Polish & Deployment

8. **Validation & Quality Gates**
   - Pre-commit validation (markdownlint, frontmatter schema, code linting)
   - CI/CD pipeline (GitHub Actions YAML with build, test, deploy jobs)
   - Manual quality gates (accessibility audit, student beta testing)

9. **Dependencies & Risks**
   - External dependencies (Algolia approval, GitHub Pages, npm registry, quiz platforms, Docker Hub)
   - Technical risks (build time, Algolia quota, quiz embed compatibility, Mermaid rendering, accessibility compliance)

10. **Success Criteria Mapping**
    - Map all 20 success criteria from spec to implementation phases
    - Show which phase delivers each SC

**Output Format:**
- Create `specs/001-book-master-plan/plan.md` following plan-template structure
- Include code samples for critical contracts (JSON Schema, TypeScript interfaces)
- Include CI/CD pipeline configuration (GitHub Actions YAML)
- Mark plan as "Ready for `/sp.tasks`" when complete

**Constitution Compliance:**
- All code examples must be tested on Ubuntu 22.04 + ROS 2 Humble (Principle 1)
- Multi-modal learning required (text, diagrams, code, exercises) (Principle 2)
- Unified terminology and glossary (Principle 3)
- 3-tier hardware support (Proxy, Miniature, Premium) (Principle 4)
- Frontmatter validation and broken link checking (Principle 5)
- Runnable code examples with Docker reproducibility (Principle 6)
```

## Response snapshot

Successfully created comprehensive implementation plan for Physical AI & Humanoid Robotics Textbook platform.

**Key Accomplishments:**

1. **Technical Context Defined**:
   - Language/Version: Docusaurus 3.0+, React 18+, Python 3.10+ (code examples), Markdown
   - Dependencies: Algolia DocSearch, Mermaid.js, ROS 2 Humble, Docker 24.0+, markdownlint, ruff
   - Performance Goals: FCP < 1.5s, LCP < 2.5s, TTI < 3.5s, Lighthouse > 90
   - Constraints: Build time < 5 min, zero broken links, WCAG 2.1 AA, Ubuntu 22.04 + ROS 2 Humble testing

2. **Constitution Check (All 6 Principles PASS)**:
   - ✅ Principle 1 (Content Accuracy): Code examples validated on Ubuntu 22.04 + ROS 2 Humble
   - ✅ Principle 2 (Educational Clarity): Multi-modal learning (text, Mermaid diagrams, code, exercises)
   - ✅ Principle 3 (Consistency): Glossary integration, unified terminology
   - ✅ Principle 4 (Lab & Hardware): 3-tier hardware guides (Proxy, Miniature, Premium)
   - ✅ Principle 5 (Docusaurus Structure): Frontmatter validation, broken link checking, search integration
   - ✅ Principle 6 (Code Quality): Runnable examples, Docker reproducibility, safety warnings

3. **Project Structure Documented**:
   - Source: docs/ (5 module directories, 30 chapters), static/ (images, downloads), src/components/ (ModuleCard, CodeBlock, QuizEmbed)
   - Configuration: docusaurus.config.js, sidebars.js, package.json, tsconfig.json
   - Contracts: contracts/ (4 files for validation)
   - Build: build/, .docusaurus/, node_modules/

4. **Phase 0: Research (8 Tasks)**:
   - R0.1: Docusaurus + React integration patterns
   - R0.2: Algolia DocSearch application process
   - R0.3: Quiz platform embed best practices (Quizlet API, Google Forms iframe)
   - R0.4: Homepage module card design patterns (grid layouts, icons, CTAs)
   - R0.5: Frontmatter validation (JSON Schema + YAML parser)
   - R0.6: Mermaid diagram accessibility (alt text, ARIA labels)
   - R0.7: Broken link checker integration (CI/CD plugin)
   - R0.8: Code example download buttons (ZIP generation, GitHub raw URLs)

5. **Phase 0.5: Prototyping Spikes (Optional, 4 Tasks)**:
   - S0.1: Algolia search prototype (basic integration, relevance tuning)
   - S0.2: Quiz embed test (iframe sandboxing, mobile responsiveness)
   - S0.3: Module cards component (interactive hover, dark mode)
   - S0.4: Frontmatter validation script (pre-commit hook)

6. **Phase 1: Design Artifacts (3 Deliverables)**:
   - data-model.md (9 entities documented: Module, Chapter, Part, Hardware Config, Code Example, Exercise, Assessment, Glossary Term, Reference)
   - contracts/ (4 files):
     - chapter-frontmatter.schema.json (JSON Schema with required fields: id, title, sidebar_label, sidebar_position, description, keywords, estimated_time, prerequisites, learning_outcomes, hardware_tier)
     - sidebar-structure.ts (TypeScript interface for sidebars.js)
     - module-card.interface.ts (TypeScript interface: moduleId, title, weekRange, description, hardwareTier, icon, chapterCount, ctaLink)
     - hardware-config.schema.json (JSON Schema for hardware tiers)
   - quickstart.md (developer onboarding: prereqs, clone, install, run, test, deploy)

7. **Phase 2: Implementation (10 Modular Phases)**:
   - **2.0: Foundational Setup** (Docusaurus init, dependencies, basic config, sidebars.js)
   - **2.1: Homepage & Navigation** (User Story 1 - P1: module cards, breadcrumbs, progress tracking)
   - **2.2: Chapter Template** (User Story 2 - P1: frontmatter, prerequisites, learning outcomes, estimated time)
   - **2.3: Code Examples** (User Story 3 - P1: syntax highlighting, copy button, download ZIP, run instructions)
   - **2.4: Diagrams** (User Story 4 - P2: Mermaid integration, flowcharts, sequence diagrams, accessibility)
   - **2.5: Exercises** (User Story 5 - P2: hands-on tasks, solutions, validation scripts)
   - **2.6: Hardware Guide** (User Story 6 - P2: 3-tier setup instructions, troubleshooting)
   - **2.7: Search Integration** (User Story 7 - P3: Algolia DocSearch application, crawler config)
   - **2.8: Assessment Checkpoints** (User Story 8 - P3: QuizEmbed component, iframe sandboxing, passing score display)
   - **2.9: Glossary & Resources** (FR-009: glossary page, reference citations)
   - **2.10: Polish & Deployment** (GitHub Pages, custom domain, versioning, analytics)

8. **CI/CD Pipeline (GitHub Actions)**:
   ```yaml
   jobs:
     build-and-test:
       steps:
         - Lint Markdown (markdownlint)
         - Validate frontmatter (JSON Schema validation script)
         - Build Docusaurus (npm run build)
         - Check broken links (broken-link-checker)
         - Accessibility audit (axe DevTools)
     deploy:
       steps:
         - Build site
         - Deploy to GitHub Pages
   ```

9. **Validation & Quality Gates**:
   - Pre-commit: markdownlint, frontmatter validation, ruff (Python)
   - CI/CD: Zero warnings/errors, broken link checking, accessibility audit
   - Manual: Student beta testing, instructor review, hardware validation

10. **Dependencies & Risks Analyzed**:
    - **External Dependencies** (5):
      - Algolia DocSearch approval (mitigation: apply early, fallback to local plugin)
      - GitHub Pages (mitigation: backup to Netlify/Vercel)
      - npm registry (mitigation: package-lock.json, mirror)
      - Quiz platforms (mitigation: standardize on 2 platforms)
      - Docker Hub (mitigation: private registry)
    - **Technical Risks** (5):
      - Build time (mitigation: incremental builds, code splitting)
      - Algolia quota (mitigation: monitor usage, optimize crawler)
      - Quiz embed compatibility (mitigation: test on mobile browsers)
      - Mermaid rendering (mitigation: static SVG fallback)
      - Accessibility compliance (mitigation: automated audits, manual testing)

11. **Success Criteria Mapping** (All 20 SC Mapped to Phases):
    - SC-001 (90% find chapter <30s) → Phase 2.1 (Navigation)
    - SC-002 (All chapters have prerequisites/outcomes) → Phase 2.2 (Chapter Template)
    - SC-003 (0 broken links) → Phase 2.10 (CI/CD)
    - SC-004-020 mapped to corresponding implementation phases

**Files Created**:
- `specs/001-book-master-plan/plan.md` (comprehensive implementation plan, ready for `/sp.tasks`)

**Constitution Alignment**: All 6 principles validated with PASS status.

**Next Steps**:
1. Execute Phase 0 Research → Generate `research.md`
2. (Optional) Execute Phase 0.5 Spikes → Validate critical unknowns
3. Execute Phase 1 Design → Generate `data-model.md`, `contracts/`, `quickstart.md`
4. Run `/sp.tasks` → Generate detailed task breakdown
5. Implement MVP (P1 User Stories) → Navigation, Prerequisites, Code Examples

## Outcome

- ✅ Impact: Comprehensive implementation plan created with modular phases (Phase 0: Research, Phase 0.5: Spikes, Phase 1: Design, Phase 2: 10 Implementation Phases) aligned with 8 user stories (P1-P3 prioritization), 34 functional requirements, and all 6 constitution principles
- 🧪 Tests: CI/CD pipeline defined with automated validation (markdownlint, frontmatter schema, build checks, broken links, accessibility audits) and manual quality gates (beta testing, instructor review)
- 📁 Files: Created `specs/001-book-master-plan/plan.md` with complete implementation strategy including JSON Schema contracts, TypeScript interfaces, GitHub Actions configuration, and success criteria mapping
- 🔁 Next prompts: Ready for Phase 0 research (`research.md` generation), Phase 1 design (`data-model.md`, `contracts/` creation), or `/sp.tasks` command to generate detailed task breakdown with dependency ordering
- 🧠 Reflection: Plan successfully bridges specification to implementation with incremental value delivery (P1 MVP first). Constitution compliance validated for all principles. External dependencies and technical risks identified with mitigation strategies. 10 modular implementation phases enable parallel work and clear checkpoints. Success criteria mapped to phases ensure measurable outcomes.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell script unavailable (`/usr/bin/env: 'pwsh': No such file or directory`), proceeded manually without setup-plan script
- Graders run and results (PASS/FAIL): PASS - All requirements met: technical context defined, constitution check completed (6/6 PASS), project structure documented, Phase 0 research tasks (8), Phase 0.5 spikes (4 optional), Phase 1 design artifacts (3), Phase 2 implementation phases (10), validation gates defined, CI/CD pipeline configured, dependencies analyzed (5 external), risks analyzed (5 technical), success criteria mapped (20/20)
- Prompt variant (if applicable): Standard `/sp.plan` workflow with enhanced modular phasing (Phase 0, 0.5, 1, 2 breakdown) adapted for educational textbook platform
- Next experiment (smallest change to try): Execute Phase 0 research to generate `research.md` with documented technology decisions (8 research tasks), then proceed to Phase 1 design artifacts or directly to `/sp.tasks` for detailed task breakdown
