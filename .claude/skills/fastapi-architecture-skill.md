# fastapi-architecture-skill

## Description

Provides reusable FastAPI architecture patterns and best practices for backend development. This skill can be leveraged by Backend Architecture Agent and API Designer to ensure consistent, scalable, and maintainable FastAPI applications.

## Components

### Clean Project Structure
- Organized folder hierarchies
- Separation of concerns
- Module organization patterns
- Configuration management
- Environment-specific settings

### Folder Layouts
- Standard directory structures
- Feature-based vs layer-based organization
- Scalable patterns for growing applications
- Test directory organization
- Static assets and templates placement

### Async Best Practices
- Proper async/await usage
- Database connection pooling
- Async dependency injection
- Background task handling
- Concurrent request processing

### Dependency Injection Patterns
- FastAPI Depends() usage
- Database session management
- Authentication dependencies
- Service layer injection
- Configuration injection

## Responsibilities

### Design Clean FastAPI Project Structures

**Recommended Project Structure (Medium to Large Applications):**

```
project_root/
├── app/
│   ├── __init__.py
│   ├── main.py                 # FastAPI app initialization
│   ├── config.py               # Configuration management
│   ├── dependencies.py         # Shared dependencies
│   │
│   ├── api/                    # API layer
│   │   ├── __init__.py
│   │   ├── routes/             # Route definitions
│   │   │   ├── __init__.py
│   │   │   ├── users.py
│   │   │   ├── content.py
│   │   │   ├── chatbot.py
│   │   │   └── auth.py
│   │   │
│   │   └── deps.py             # API-specific dependencies
│   │
│   ├── core/                   # Core functionality
│   │   ├── __init__.py
│   │   ├── config.py           # Settings and configuration
│   │   ├── security.py         # Security utilities (JWT, password hashing)
│   │   └── logging.py          # Logging configuration
│   │
│   ├── models/                 # Database models (SQLAlchemy/Pydantic)
│   │   ├── __init__.py
│   │   ├── user.py
│   │   ├── content.py
│   │   └── progress.py
│   │
│   ├── schemas/                # Pydantic schemas (request/response)
│   │   ├── __init__.py
│   │   ├── user.py
│   │   ├── content.py
│   │   ├── chatbot.py
│   │   └── auth.py
│   │
│   ├── services/               # Business logic layer
│   │   ├── __init__.py
│   │   ├── user_service.py
│   │   ├── content_service.py
│   │   ├── rag_service.py
│   │   └── auth_service.py
│   │
│   ├── repositories/           # Data access layer
│   │   ├── __init__.py
│   │   ├── user_repository.py
│   │   ├── content_repository.py
│   │   └── base_repository.py
│   │
│   ├── db/                     # Database configuration
│   │   ├── __init__.py
│   │   ├── session.py          # Database session management
│   │   ├── base.py             # Base model classes
│   │   └── migrations/         # Alembic migrations (if using)
│   │
│   ├── utils/                  # Utility functions
│   │   ├── __init__.py
│   │   ├── helpers.py
│   │   └── validators.py
│   │
│   └── tests/                  # Tests
│       ├── __init__.py
│       ├── conftest.py         # Pytest fixtures
│       ├── test_api/
│       ├── test_services/
│       └── test_repositories/
│
├── alembic/                    # Database migrations (if using Alembic)
│   └── versions/
│
├── scripts/                    # Utility scripts
│   ├── seed_data.py
│   └── generate_embeddings.py
│
├── .env                        # Environment variables (not committed)
├── .env.example                # Example environment file
├── requirements.txt            # Dependencies
├── pyproject.toml              # Project metadata (if using)
├── pytest.ini                  # Pytest configuration
├── docker-compose.yml          # Docker setup
├── Dockerfile
└── README.md
```

**Small Application Structure:**

```
project_root/
├── app/
│   ├── main.py                 # Everything in one file for small apps
│   ├── database.py
│   ├── models.py
│   ├── schemas.py
│   └── config.py
├── .env
├── requirements.txt
└── README.md
```

**Key Principles:**
1. **Separation of Concerns**: API routes, business logic, and data access separated
2. **Dependency Direction**: Dependencies flow inward (routes → services → repositories)
3. **Scalability**: Easy to add new features without restructuring
4. **Testability**: Each layer can be tested independently

### Suggest Folder Layouts

**Feature-Based Organization (Alternative for Large Apps):**

```
app/
├── main.py
├── config.py
├── dependencies.py
│
├── features/
│   ├── users/
│   │   ├── __init__.py
│   │   ├── routes.py
│   │   ├── schemas.py
│   │   ├── models.py
│   │   ├── service.py
│   │   └── repository.py
│   │
│   ├── content/
│   │   ├── __init__.py
│   │   ├── routes.py
│   │   ├── schemas.py
│   │   ├── models.py
│   │   ├── service.py
│   │   └── repository.py
│   │
│   └── chatbot/
│       ├── __init__.py
│       ├── routes.py
│       ├── schemas.py
│       ├── service.py
│       └── rag_pipeline.py
│
└── core/
    ├── security.py
    ├── database.py
    └── config.py
```

**When to Use Feature-Based:**
- Large applications with many distinct features
- Teams organized by feature
- Features with minimal cross-dependencies
- Microservices-like architecture within monolith

**When to Use Layer-Based:**
- Medium-sized applications
- Shared business logic across features
- Need clear separation of concerns
- Traditional MVC background

### Enforce Async Best Practices

**1. Database Operations:**

```python
# ✅ GOOD: Async database operations
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker

# Create async engine
engine = create_async_engine(
    "postgresql+asyncpg://user:pass@localhost/db",
    echo=True,
    future=True,
    pool_size=20,
    max_overflow=0
)

# Create async session maker
async_session = sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)

# Use in routes
@app.get("/users/{user_id}")
async def get_user(user_id: int, db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(User).where(User.id == user_id))
    user = result.scalar_one_or_none()
    return user

# ❌ BAD: Blocking database operations
def get_user_sync(user_id: int, db: Session = Depends(get_db)):
    user = db.query(User).filter(User.id == user_id).first()
    return user
```

**2. External API Calls:**

```python
# ✅ GOOD: Async HTTP requests
import httpx

async def fetch_external_data(url: str):
    async with httpx.AsyncClient() as client:
        response = await client.get(url)
        return response.json()

# ❌ BAD: Sync requests blocking event loop
import requests

def fetch_external_data_sync(url: str):
    response = requests.get(url)  # Blocks!
    return response.json()
```

**3. Background Tasks:**

```python
from fastapi import BackgroundTasks

# ✅ GOOD: Use BackgroundTasks for non-critical operations
@app.post("/users/")
async def create_user(
    user: UserCreate,
    background_tasks: BackgroundTasks,
    db: AsyncSession = Depends(get_db)
):
    # Critical: Create user (awaited)
    new_user = await user_service.create_user(db, user)

    # Non-critical: Send email (background)
    background_tasks.add_task(send_welcome_email, new_user.email)

    return new_user

# For longer tasks, use Celery or similar
```

**4. Concurrent Operations:**

```python
import asyncio

# ✅ GOOD: Run independent operations concurrently
async def get_dashboard_data(user_id: int, db: AsyncSession):
    # Run multiple queries concurrently
    user_task = get_user(db, user_id)
    progress_task = get_user_progress(db, user_id)
    stats_task = get_user_stats(db, user_id)

    user, progress, stats = await asyncio.gather(
        user_task,
        progress_task,
        stats_task
    )

    return {"user": user, "progress": progress, "stats": stats}

# ❌ BAD: Sequential operations
async def get_dashboard_data_slow(user_id: int, db: AsyncSession):
    user = await get_user(db, user_id)
    progress = await get_user_progress(db, user_id)
    stats = await get_user_stats(db, user_id)
    return {"user": user, "progress": progress, "stats": stats}
```

**5. Connection Pooling:**

```python
# ✅ GOOD: Proper connection pool configuration
engine = create_async_engine(
    DATABASE_URL,
    pool_size=20,           # Number of connections to maintain
    max_overflow=10,        # Additional connections if pool exhausted
    pool_timeout=30,        # Wait time before raising error
    pool_recycle=3600,      # Recycle connections every hour
    pool_pre_ping=True      # Verify connections before use
)
```

**Async Guidelines:**
- Use `async def` for all route handlers
- Always `await` async operations
- Use async libraries (asyncpg, httpx, motor)
- Avoid blocking operations in async functions
- Use `run_in_executor` for CPU-bound operations
- Configure connection pooling appropriately

### Define Dependency Injection Patterns

**1. Database Session Dependency:**

```python
# app/db/session.py
from sqlalchemy.ext.asyncio import AsyncSession
from app.db.base import async_session

async def get_db() -> AsyncSession:
    """Dependency for database session"""
    async with async_session() as session:
        try:
            yield session
            await session.commit()
        except Exception:
            await session.rollback()
            raise
        finally:
            await session.close()

# Usage in routes
@app.get("/users/{user_id}")
async def get_user(
    user_id: int,
    db: AsyncSession = Depends(get_db)
):
    return await user_service.get_user(db, user_id)
```

**2. Authentication Dependency:**

```python
# app/api/deps.py
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from jose import JWTError, jwt

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

async def get_current_user(
    token: str = Depends(oauth2_scheme),
    db: AsyncSession = Depends(get_db)
) -> User:
    """Get current authenticated user"""
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id: str = payload.get("sub")
        if user_id is None:
            raise credentials_exception
    except JWTError:
        raise credentials_exception

    user = await user_service.get_user(db, int(user_id))
    if user is None:
        raise credentials_exception

    return user

# Usage: Protect routes
@app.get("/users/me")
async def read_users_me(
    current_user: User = Depends(get_current_user)
):
    return current_user

# Chain dependencies
async def get_current_active_user(
    current_user: User = Depends(get_current_user)
) -> User:
    """Ensure user is active"""
    if not current_user.is_active:
        raise HTTPException(status_code=400, detail="Inactive user")
    return current_user
```

**3. Service Layer Dependency:**

```python
# app/services/user_service.py
class UserService:
    def __init__(self, repository: UserRepository):
        self.repository = repository

    async def get_user(self, db: AsyncSession, user_id: int):
        return await self.repository.get_by_id(db, user_id)

    async def create_user(self, db: AsyncSession, user_data: UserCreate):
        # Business logic here
        return await self.repository.create(db, user_data)

# app/api/deps.py
def get_user_service() -> UserService:
    """Dependency for user service"""
    return UserService(UserRepository())

# Usage
@app.get("/users/{user_id}")
async def get_user(
    user_id: int,
    db: AsyncSession = Depends(get_db),
    user_service: UserService = Depends(get_user_service)
):
    return await user_service.get_user(db, user_id)
```

**4. Configuration Dependency:**

```python
# app/core/config.py
from pydantic_settings import BaseSettings
from functools import lru_cache

class Settings(BaseSettings):
    app_name: str = "My App"
    database_url: str
    secret_key: str
    openai_api_key: str

    class Config:
        env_file = ".env"

@lru_cache()
def get_settings() -> Settings:
    """Cache settings for performance"""
    return Settings()

# Usage
@app.get("/info")
async def info(settings: Settings = Depends(get_settings)):
    return {"app_name": settings.app_name}
```

**5. Common Dependency Patterns:**

```python
# Pagination dependency
class PaginationParams:
    def __init__(self, skip: int = 0, limit: int = 100):
        self.skip = skip
        self.limit = min(limit, 100)  # Cap at 100

@app.get("/users")
async def list_users(
    pagination: PaginationParams = Depends(),
    db: AsyncSession = Depends(get_db)
):
    users = await user_service.get_users(
        db,
        skip=pagination.skip,
        limit=pagination.limit
    )
    return users

# Query parameter dependency
class UserFilters:
    def __init__(
        self,
        grade_level: Optional[str] = None,
        is_active: Optional[bool] = None,
        search: Optional[str] = None
    ):
        self.grade_level = grade_level
        self.is_active = is_active
        self.search = search

@app.get("/users/search")
async def search_users(
    filters: UserFilters = Depends(),
    db: AsyncSession = Depends(get_db)
):
    return await user_service.search_users(db, filters)
```

**6. Role-Based Access Control:**

```python
from enum import Enum

class UserRole(str, Enum):
    ADMIN = "admin"
    TEACHER = "teacher"
    STUDENT = "student"

def require_role(required_role: UserRole):
    """Dependency factory for role-based access"""
    async def role_checker(
        current_user: User = Depends(get_current_user)
    ):
        if current_user.role != required_role:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Insufficient permissions"
            )
        return current_user
    return role_checker

# Usage
@app.delete("/users/{user_id}")
async def delete_user(
    user_id: int,
    db: AsyncSession = Depends(get_db),
    admin: User = Depends(require_role(UserRole.ADMIN))
):
    await user_service.delete_user(db, user_id)
    return {"message": "User deleted"}
```

## Best Practices

### 1. Error Handling

```python
# Custom exception handlers
from fastapi import Request
from fastapi.responses import JSONResponse

class CustomException(Exception):
    def __init__(self, message: str, status_code: int = 400):
        self.message = message
        self.status_code = status_code

@app.exception_handler(CustomException)
async def custom_exception_handler(request: Request, exc: CustomException):
    return JSONResponse(
        status_code=exc.status_code,
        content={"message": exc.message}
    )

# Use in services
async def get_user(db: AsyncSession, user_id: int):
    user = await db.execute(select(User).where(User.id == user_id))
    user = user.scalar_one_or_none()

    if user is None:
        raise CustomException(
            message=f"User {user_id} not found",
            status_code=404
        )

    return user
```

### 2. Response Models

```python
# Always use response_model for type safety and documentation
@app.get("/users/{user_id}", response_model=UserResponse)
async def get_user(user_id: int, db: AsyncSession = Depends(get_db)):
    return await user_service.get_user(db, user_id)

# For lists
@app.get("/users", response_model=List[UserResponse])
async def list_users(db: AsyncSession = Depends(get_db)):
    return await user_service.get_all_users(db)

# With status codes
@app.post("/users", response_model=UserResponse, status_code=201)
async def create_user(user: UserCreate, db: AsyncSession = Depends(get_db)):
    return await user_service.create_user(db, user)
```

### 3. API Versioning

```python
# Option 1: URL versioning
api_v1 = APIRouter(prefix="/api/v1")
api_v2 = APIRouter(prefix="/api/v2")

@api_v1.get("/users")
async def get_users_v1():
    pass

@api_v2.get("/users")
async def get_users_v2():
    pass

app.include_router(api_v1)
app.include_router(api_v2)

# Option 2: Header versioning
@app.get("/users")
async def get_users(api_version: str = Header(default="v1")):
    if api_version == "v1":
        return get_users_v1()
    elif api_version == "v2":
        return get_users_v2()
```

### 4. Middleware

```python
import time
from fastapi import Request

@app.middleware("http")
async def add_process_time_header(request: Request, call_next):
    start_time = time.time()
    response = await call_next(request)
    process_time = time.time() - start_time
    response.headers["X-Process-Time"] = str(process_time)
    return response

# CORS
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### 5. Logging

```python
import logging
from app.core.config import get_settings

settings = get_settings()

logging.basicConfig(
    level=logging.INFO if settings.environment == "production" else logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

logger = logging.getLogger(__name__)

@app.get("/users/{user_id}")
async def get_user(user_id: int, db: AsyncSession = Depends(get_db)):
    logger.info(f"Fetching user {user_id}")
    try:
        user = await user_service.get_user(db, user_id)
        return user
    except Exception as e:
        logger.error(f"Error fetching user {user_id}: {str(e)}")
        raise
```

## Usage Guidelines

When this skill is invoked, agents should:

1. **Start with clear structure** - Choose appropriate folder layout for project size
2. **Enforce async patterns** - All I/O operations must be async
3. **Use dependency injection** - Leverage FastAPI's Depends() for clean code
4. **Separate concerns** - Keep routes, services, and data access separate
5. **Document APIs** - Use Pydantic models and docstrings
6. **Handle errors gracefully** - Implement proper exception handling
7. **Test thoroughly** - Write tests for each layer

## Integration with Other Skills

- **RAG Design**: Structure RAG services following these patterns
- **Vector Database (Qdrant)**: Inject Qdrant client as dependency
- **Auth & User Context**: Implement auth dependencies as shown
- **Content Personalization**: Use dependencies to inject user context

This skill ensures consistent, maintainable, and scalable FastAPI applications across the entire project.
