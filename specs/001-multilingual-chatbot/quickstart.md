# Quickstart Guide: Multilingual Docusaurus + RAG Chatbot

## Prerequisites
- Node.js 18+ for Docusaurus frontend
- Python 3.10+ for FastAPI backend
- OpenRouter API key for translation services
- Git for version control

## Setup

### 1. Environment Configuration
```bash
# Clone the repository
git clone <repository-url>
cd ai-and-humanoid-robotics-hackathon

# Set up environment variables
cp .env.example .env
# Edit .env and add your OPENROUTER_API_KEY
```

### 2. Backend Setup
```bash
# Navigate to backend directory (or use existing structure)
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install fastapi uvicorn python-multipart

# Install additional dependencies for translation
pip install openrouter-python httpx
```

### 3. Frontend Setup
```bash
# Install Node.js dependencies
npm install

# For Docusaurus i18n setup
npm run write-translations -- --locale ur
npm run write-translations -- --locale ar
# Repeat for other languages as needed
```

## Running the Application

### 1. Start the Backend
```bash
# From the backend directory
uvicorn main:app --reload --port 8000
```

### 2. Start the Frontend (Docusaurus)
```bash
# From the project root
npm start
```

### 3. Access the Application
- Documentation: `http://localhost:3000`
- Documentation (Urdu): `http://localhost:3000/ur`
- Documentation (Arabic): `http://localhost:3000/ar`
- Backend API: `http://localhost:8000`
- Translation API: `http://localhost:8000/api/v1/translate`

## Translation API Usage

### POST /api/v1/translate
```bash
curl -X POST "http://localhost:8000/api/v1/translate" \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Hello, world!",
    "target_language": "ur",
    "source_language": "en",
    "context": "greeting"
  }'
```

## Docusaurus i18n Commands

### Add a new language
```bash
# Update docusaurus.config.js to add the new locale
# Then generate UI translations:
npm run write-translations -- --locale [new-locale-code]

# Copy English docs to new locale:
cp -r docs i18n/[new-locale-code]/docusaurus-plugin-content-docs/current/
```

### Build for specific locale
```bash
npm run build -- --locale ur
```

### Build all locales
```bash
npm run build:all
```

## Testing

### Backend Tests
```bash
# Run backend tests
python -m pytest tests/
```

### Frontend Tests
```bash
# Run frontend tests
npm test
```

## Configuration

### Environment Variables
- `OPENROUTER_API_KEY`: API key for OpenRouter translation service
- `OPENROUTER_MODEL`: Model to use (default: `mistralai/devstral-2512:free`)
- `CACHE_TTL`: Translation cache TTL in seconds (default: 900 for 15 minutes)
- `RATE_LIMIT_PER_IP`: Requests per minute per IP (default: 100)
- `RATE_LIMIT_PER_SESSION`: Requests per minute per session (default: 50)

### Docusaurus Configuration
The `docusaurus.config.js` file includes:
- i18n configuration for 8 supported languages
- Locale-specific settings for RTL languages
- hreflang tags for SEO
- Language switcher in navbar

## Troubleshooting

### Common Issues
1. **Translation API errors**: Verify `OPENROUTER_API_KEY` is set correctly
2. **RTL layout issues**: Check that CSS uses logical properties instead of left/right
3. **Font rendering problems**: Ensure Noto fonts are properly loaded for RTL languages
4. **Language switching delays**: Verify cache is working properly

### Debugging Translation API
```bash
# Enable debug logging
export LOG_LEVEL=DEBUG
uvicorn main:app --reload --port 8000
```