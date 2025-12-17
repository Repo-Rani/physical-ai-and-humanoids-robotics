# Research: Docusaurus Urdu Translation Implementation

## Decision: Translation Approach
**Rationale**: To translate Docusaurus documentation from English to Urdu while maintaining technical accuracy, we'll use Docusaurus's built-in i18n (internationalization) system which provides proper language switching capabilities.

## Technical Implementation
**Approach**:
1. Set up Docusaurus i18n configuration to support Urdu language
2. Create Urdu translation files in the i18n/ur/ directory structure
3. Translate content while preserving technical terms and code blocks
4. Configure language switcher in the navigation bar

## Alternatives Considered
1. **Separate documentation site**: Would create maintenance burden and fragment content
2. **Manual translation without i18n**: Would not provide proper language switching functionality
3. **Machine translation only**: Would not maintain technical accuracy required by constitution
4. **Dual-language content**: Would clutter the interface and not provide clean language separation

## Docusaurus i18n Configuration
**Required Changes**:
- Update `docusaurus.config.js` to include Urdu as a supported language
- Create directory structure: `i18n/ur/docusaurus-plugin-content-docs/current/`
- Generate Urdu translation files that mirror the English structure
- Configure language switcher in navbar

## Technical Term Handling
**Decision**: Technical terms like "actuator", "servo motor", "sensor", ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA will remain in English as per constitution principle of maintaining technical accuracy. Explanatory text will be translated to Urdu.

## File Structure Mapping
**English → Urdu mapping**:
- `docs/intro.md` → `i18n/ur/docusaurus-plugin-content-docs/current/intro.md`
- `docs/module-1-ros2/` → `i18n/ur/docusaurus-plugin-content-docs/current/module-1-ros2/`
- And so on for all documentation files and directories

## Validation Strategy
1. Verify all translated pages maintain original formatting and structure
2. Test language switching functionality works correctly
3. Confirm technical terminology accuracy is maintained
4. Ensure all navigation and cross-references work in Urdu version