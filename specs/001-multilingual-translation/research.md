# Research: Multi-Language Translation System

## Phase 0: Resolving Unknowns and Best Practices

### Decision: Fallback Behavior for Missing Translations
**Rationale**: When a user switches to a language where a specific page doesn't have a translation, the system should gracefully handle this scenario. The best practice is to fall back to the English version of the page while indicating to the user that the translation is not yet available. This ensures users can still access the content while being aware of the limitation.

**Alternatives considered**:
- Show an error page: Would create a poor user experience
- Show a notification only: Doesn't provide the content the user is looking for
- Redirect to home page: Loses the user's context

**Chosen approach**: Display the English version of the page with a notification that the translation is not yet available, allowing users to continue learning while being aware of the translation status.

### Decision: Docusaurus i18n Implementation Approach
**Rationale**: Docusaurus provides built-in internationalization support through its i18n plugin system. This approach is well-documented, maintainable, and integrates seamlessly with the existing documentation structure. It allows for proper URL routing, navigation persistence, and search functionality across languages.

**Alternatives considered**:
- Custom language switching: Would require building routing and state management from scratch
- Third-party translation libraries: Would add unnecessary complexity and dependencies
- Server-side language detection: Not needed for static documentation site

**Chosen approach**: Use Docusaurus built-in i18n plugin with locale configuration for English (en), Urdu (ur), and Hindi (hi).

### Decision: RTL Support for Urdu
**Rationale**: Urdu is a right-to-left (RTL) language and requires proper text rendering, alignment, and navigation flow. Docusaurus supports RTL through CSS and HTML direction attributes. This is essential for proper readability and user experience for Urdu speakers.

**Alternatives considered**:
- Left-align Urdu text: Would be incorrect and difficult to read
- Mixed LTR/RTL without proper isolation: Would cause layout issues
- Separate RTL styling system: Would create inconsistency

**Chosen approach**: Implement RTL support using CSS `[dir='rtl']` selectors and proper HTML direction attributes as per Docusaurus guidelines.

### Decision: Language Switching UI Components
**Rationale**: The system needs both global (navbar) and local (chapter-level) language switching options. The navbar dropdown provides site-wide language selection, while the chapter-level button allows granular control. Both should maintain consistent styling with the existing orange gradient theme.

**Alternatives considered**:
- Only navbar dropdown: Would limit user flexibility
- Only chapter-level buttons: Would make site-wide switching cumbersome
- Modal language picker: Would interrupt user flow

**Chosen approach**: Implement both navbar dropdown and chapter-level TranslateButton with consistent styling.

### Decision: Translation File Structure
**Rationale**: Following Docusaurus i18n conventions ensures proper integration with the build system and navigation. The directory structure should mirror the English content structure to maintain consistency and simplify maintenance.

**Alternatives considered**:
- Flattened translation structure: Would break navigation and linking
- Different directory naming: Would not follow Docusaurus conventions
- Single translation files: Would make management difficult

**Chosen approach**: Mirror English content structure in `/i18n/{locale}/docusaurus-plugin-content-docs/current/` directories.