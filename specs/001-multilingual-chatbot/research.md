# Research Summary: Multilingual Docusaurus + RAG Chatbot

## Decision: Technology Stack Selection
**Rationale**: Selected FastAPI backend with Docusaurus frontend based on the multilingual chatbot constitution requirements. FastAPI provides excellent async support for translation API endpoints and integrates well with OpenRouter. Docusaurus has built-in i18n support that aligns with the constitution's requirements for proper SEO and navigation consistency.

**Alternatives considered**:
- Next.js + API routes: More complex for documentation-focused site
- Pure static site with client-side translation: Would not meet performance requirements
- Custom backend framework: Would not align with constitution's FastAPI requirement

## Decision: Translation API Implementation
**Rationale**: Using OpenRouter API with `mistralai/devstral-2512:free` model as specified in the constitution. Implementation includes in-memory caching with 15-minute TTL to meet the 70%+ cache hit rate requirement and reduce API costs. Rate limiting is implemented per constitution specifications (100/min per IP, 50/min per session).

**Alternatives considered**:
- Google Cloud Translation: Would incur costs that exceed free tier
- Local translation models: Would not meet performance requirements for real-time chat
- Other LLM providers: Constitution specifically mandates OpenRouter

## Decision: RTL Layout Implementation
**Rationale**: Using CSS logical properties (margin-inline-start, padding-inline-end, etc.) as specified in the constitution to ensure proper RTL layout for Urdu and Arabic. This approach provides proper mirroring of layouts without hardcoding directional properties and maintains consistency across all UI components.

**Alternatives considered**:
- CSS left/right properties with RTL detection: Would not meet constitution's "NO hardcoded directional properties" requirement
- Separate RTL CSS files: Would create maintenance overhead
- JavaScript-based RTL handling: Would not be as performant as CSS-based approach

## Decision: Message Storage and Translation State
**Rationale**: Using localStorage for client-side message storage with the schema specified in the constitution. Each message includes both original and translated text to enable language switching without re-translation. This approach preserves conversation history and meets the requirement for instant language switching.

**Alternatives considered**:
- Server-side storage: Would create privacy concerns and additional complexity
- Session storage: Would not persist across browser sessions
- Cookies: Would be limited in storage capacity for long conversations

## Decision: Font Selection for RTL Languages
**Rationale**: Using Noto fonts as specified in the constitution (Noto Nastaliq Urdu for Urdu, Noto Sans Arabic for Arabic) with proper fallback chains. These fonts provide excellent support for RTL scripts and are available as web fonts for consistent rendering across platforms.

**Alternatives considered**:
- System fonts only: Would not guarantee proper rendering on all systems
- Custom web fonts: Would not align with constitution's Noto font requirement
- Image-based text rendering: Would not be accessible or SEO-friendly