# auth-user-context-skill

## Description

Provides reusable guidance for authentication flows and safe user-context handling.

## Components

### Signup / Signin Flow Design
- Registration workflow with validation
- Login mechanisms (email/password, OAuth, SSO)
- Password requirements and hashing strategies
- Email verification and account activation
- Multi-factor authentication (MFA) integration
- Session management and token lifecycle
- Password reset and account recovery flows

### User Background Data Modeling
- Profile information structure (name, email, preferences)
- Educational background (grade level, subjects, interests)
- Learning preferences and accessibility needs
- Progress tracking and achievement data
- Privacy-sensitive information handling
- Consent and data collection policies
- Data retention and deletion strategies

### Context Exposure Rules
- What user data can be shared with AI agents
- What data should remain private or encrypted
- Context scoping for different agent types
- Session-based vs persistent context
- Data anonymization for analytics
- Cross-agent context propagation rules
- Audit trails for context access

## Responsibilities

### Guide Secure Authentication Architecture

**Authentication Mechanisms:**
- **Email/Password**: Implement secure password hashing (bcrypt, argon2)
- **OAuth 2.0**: Support social login (Google, GitHub, Microsoft)
- **JWT Tokens**: Stateless authentication with proper expiration
- **Refresh Tokens**: Secure token refresh without re-authentication
- **Session Management**: Server-side session storage and validation
- **MFA/2FA**: Time-based OTP or SMS verification for enhanced security

**Security Best Practices:**
- Never store passwords in plain text
- Use HTTPS for all authentication endpoints
- Implement rate limiting on login attempts
- Use secure, httpOnly cookies for session tokens
- Implement CSRF protection for state-changing operations
- Validate and sanitize all user inputs
- Log authentication events for security monitoring

**Flow Design:**
```
Signup Flow:
1. User submits email + password
2. Validate email format and password strength
3. Check for existing account
4. Hash password using bcrypt/argon2
5. Store user record with unverified status
6. Send verification email with token
7. User clicks verification link
8. Activate account and redirect to login

Signin Flow:
1. User submits credentials
2. Rate limit check (prevent brute force)
3. Validate credentials against database
4. Check account status (verified, active, banned)
5. Generate JWT/session token
6. Set secure cookie or return token
7. Redirect to dashboard/home

Password Reset Flow:
1. User requests reset with email
2. Generate time-limited reset token
3. Send reset link via email
4. User clicks link and enters new password
5. Validate token and update password
6. Invalidate all existing sessions
7. Redirect to login with success message
```

**Token Management:**
- **Access Tokens**: Short-lived (15-30 minutes), contain user ID and roles
- **Refresh Tokens**: Long-lived (7-30 days), stored securely, rotated on use
- **Token Structure**: Use JWT with claims (sub, exp, iat, roles)
- **Token Storage**: httpOnly cookies (web) or secure storage (mobile)
- **Token Validation**: Verify signature, expiration, and revocation status

### Help Structure User Background Data

**Core User Profile:**
```json
{
  "user_id": "uuid",
  "email": "user@example.com",
  "email_verified": true,
  "name": "Full Name",
  "created_at": "ISO8601 timestamp",
  "updated_at": "ISO8601 timestamp",
  "status": "active" // active, suspended, deleted
}
```

**Educational Background:**
```json
{
  "grade_level": "high_school" // elementary, middle_school, high_school, college, professional,
  "subjects_of_interest": ["robotics", "ai", "programming"],
  "proficiency_levels": {
    "programming": "intermediate",
    "robotics": "beginner",
    "ai": "beginner"
  },
  "learning_goals": ["build_a_robot", "understand_ml"],
  "preferred_language": "en" // ISO 639-1 codes
}
```

**Learning Preferences:**
```json
{
  "learning_style": "visual", // visual, auditory, kinesthetic, reading
  "pace_preference": "self_paced", // self_paced, guided, structured
  "accessibility_needs": {
    "screen_reader": false,
    "high_contrast": false,
    "text_size": "medium"
  },
  "notification_preferences": {
    "email": true,
    "push": false,
    "frequency": "weekly"
  }
}
```

**Progress and Context:**
```json
{
  "current_chapter": "chapter-3-sensors",
  "completed_chapters": ["chapter-1-intro", "chapter-2-basics"],
  "quiz_scores": {
    "chapter-1-quiz": 85,
    "chapter-2-quiz": 92
  },
  "time_spent_minutes": 240,
  "last_active": "ISO8601 timestamp",
  "bookmarks": ["page-45", "page-67"],
  "notes_count": 12
}
```

**Privacy and Consent:**
```json
{
  "consent": {
    "terms_accepted": true,
    "privacy_policy_accepted": true,
    "analytics_consent": true,
    "marketing_consent": false,
    "consent_date": "ISO8601 timestamp"
  },
  "privacy_settings": {
    "profile_visibility": "private", // public, private, friends
    "share_progress": false,
    "allow_data_analysis": true
  }
}
```

**Data Modeling Best Practices:**
- **Separate concerns**: Split user data into logical tables/collections
- **Normalize appropriately**: Balance between normalization and query performance
- **Use references**: Link related data with foreign keys or document references
- **Index strategically**: Index fields used in frequent queries
- **Version data schemas**: Include schema version for migration support
- **Soft delete**: Use status flags instead of hard deletes for compliance

### Ensure Safe Sharing of User Context Across Agents

**Context Exposure Principles:**

**Level 1 - Public Context (Shareable with all agents):**
- User's first name (not full name)
- Grade level / learning stage
- Subjects of interest
- Current chapter/topic being studied
- Preferred language
- Learning style preference

**Level 2 - Private Context (Agent-specific, need-to-know):**
- Full name and email (only for profile/account agents)
- Progress data (only for progress tracking agents)
- Quiz scores (only for assessment agents)
- Bookmarks and notes (only for content agents)

**Level 3 - Restricted Context (Never shared with agents):**
- Password hashes
- Authentication tokens
- Payment information
- Personal identifiable information (PII) beyond name
- IP addresses and device information
- Full browsing history

**Context Scoping Rules:**

```python
# Example context scoping logic
def get_agent_context(user_id, agent_type):
    base_context = {
        "user_id": user_id,
        "first_name": get_first_name(user_id),
        "grade_level": get_grade_level(user_id),
        "subjects": get_interests(user_id),
        "current_topic": get_current_topic(user_id)
    }

    if agent_type == "chatbot":
        # Chatbot needs learning context
        base_context.update({
            "learning_style": get_learning_style(user_id),
            "proficiency": get_proficiency_levels(user_id),
            "recent_topics": get_recent_topics(user_id, limit=5)
        })

    elif agent_type == "assessment":
        # Assessment agent needs progress data
        base_context.update({
            "completed_chapters": get_completed_chapters(user_id),
            "quiz_history": get_quiz_scores(user_id),
            "weak_areas": identify_weak_areas(user_id)
        })

    # Never include sensitive data
    # No passwords, tokens, PII, payment info

    return base_context
```

**Cross-Agent Context Propagation:**
- **Session-based context**: Share context within a single user session
- **Persistent context**: Store long-term preferences and progress
- **Event-driven updates**: Propagate context changes via events/message queues
- **Context versioning**: Track context changes over time
- **Access logging**: Log which agents accessed what user data when

**Anonymization for Analytics:**
- **Hash user IDs**: Use one-way hashing for analytics
- **Aggregate data**: Report on cohorts, not individuals
- **Remove PII**: Strip names, emails before analytics processing
- **Differential privacy**: Add noise to prevent re-identification
- **Consent-based**: Only analyze data from users who opted in

## Usage Guidelines

When this skill is invoked, agents should:

1. **Prioritize security** in all authentication and data handling decisions
2. **Minimize data exposure** - share only what's necessary for each agent
3. **Respect user consent** - honor privacy preferences and consent choices
4. **Implement audit trails** - log access to sensitive user data
5. **Design for privacy** - apply privacy-by-design principles from the start

## Best Practices

### Authentication
- **Use industry-standard libraries** (Passport.js, Auth0, Firebase Auth)
- **Implement progressive profiling** - collect data over time, not all at once
- **Support account deletion** - provide users control over their data
- **Test security regularly** - conduct penetration testing and code reviews

### User Data
- **Collect only necessary data** - don't ask for information you won't use
- **Encrypt sensitive data at rest** - use database-level encryption for PII
- **Use secure transmission** - always use HTTPS/TLS for data transfer
- **Implement data retention policies** - delete or anonymize old data

### Context Management
- **Default to minimal exposure** - share less rather than more
- **Use role-based access control** - grant permissions based on agent roles
- **Implement context timeouts** - expire context after inactivity
- **Provide user visibility** - show users what data is being used by agents

### Compliance
- **GDPR compliance** - right to access, rectification, erasure, portability
- **COPPA compliance** - additional protections for users under 13
- **FERPA compliance** - protect educational records (if applicable)
- **Regular audits** - review data handling practices quarterly

## Integration Patterns

### FastAPI + JWT Example
```python
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from jose import JWTError, jwt

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

async def get_current_user(token: str = Depends(oauth2_scheme)):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id: str = payload.get("sub")
        if user_id is None:
            raise HTTPException(status_code=401)
        return get_user_by_id(user_id)
    except JWTError:
        raise HTTPException(status_code=401)

# Protected endpoint
@app.get("/api/user/profile")
async def get_profile(current_user = Depends(get_current_user)):
    return filter_user_data(current_user, level="profile_view")
```

### Context Middleware
```python
@app.middleware("http")
async def add_user_context(request: Request, call_next):
    # Extract user from token
    user = await get_user_from_request(request)

    # Build appropriate context based on endpoint
    agent_type = determine_agent_type(request.url.path)
    context = get_agent_context(user.id, agent_type)

    # Attach to request state
    request.state.user_context = context

    # Log context access
    log_context_access(user.id, agent_type, request.url.path)

    response = await call_next(request)
    return response
```
