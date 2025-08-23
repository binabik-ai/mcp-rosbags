#!/usr/bin/env python3
"""
Cache manager for efficient message retrieval.
"""

import hashlib
import json
import time
from typing import Any, Dict, Optional
from collections import OrderedDict
import logging

logger = logging.getLogger(__name__)


class CacheManager:
    """LRU cache with TTL for message queries."""
    
    def __init__(self, max_size: int = 100, ttl_seconds: int = 300):
        self.max_size = max_size
        self.ttl_seconds = ttl_seconds
        self.cache: OrderedDict[str, tuple[Any, float]] = OrderedDict()
        self.hits = 0
        self.misses = 0
    
    def get_key(self, tool_name: str, arguments: Dict[str, Any]) -> str:
        """Generate cache key from tool name and arguments."""
        # Create a deterministic string representation
        key_parts = [tool_name]
        for k in sorted(arguments.keys()):
            key_parts.append(f"{k}:{arguments[k]}")
        
        key_str = "|".join(key_parts)
        return hashlib.md5(key_str.encode()).hexdigest()
    
    def get(self, key: str) -> Optional[Any]:
        """Get value from cache if not expired."""
        if key not in self.cache:
            self.misses += 1
            return None
        
        value, timestamp = self.cache[key]
        
        # Check if expired
        if time.time() - timestamp > self.ttl_seconds:
            del self.cache[key]
            self.misses += 1
            return None
        
        # Move to end (most recently used)
        self.cache.move_to_end(key)
        self.hits += 1
        
        logger.debug(f"Cache hit for key {key[:8]}... (hit rate: {self.get_hit_rate():.1%})")
        return value
    
    def set(self, key: str, value: Any):
        """Set value in cache with current timestamp."""
        # Remove oldest if at capacity
        if len(self.cache) >= self.max_size and key not in self.cache:
            oldest_key = next(iter(self.cache))
            del self.cache[oldest_key]
            logger.debug(f"Evicted oldest cache entry: {oldest_key[:8]}...")
        
        self.cache[key] = (value, time.time())
        self.cache.move_to_end(key)
    
    def clear(self):
        """Clear all cache entries."""
        size = len(self.cache)
        self.cache.clear()
        self.hits = 0
        self.misses = 0
        logger.info(f"Cleared {size} cache entries")
    
    def get_stats(self) -> Dict[str, Any]:
        """Get cache statistics."""
        total_requests = self.hits + self.misses
        return {
            "size": len(self.cache),
            "max_size": self.max_size,
            "hits": self.hits,
            "misses": self.misses,
            "hit_rate": self.hits / total_requests if total_requests > 0 else 0,
            "ttl_seconds": self.ttl_seconds
        }
    
    def get_hit_rate(self) -> float:
        """Get cache hit rate."""
        total = self.hits + self.misses
        return self.hits / total if total > 0 else 0
    
    def cleanup_expired(self):
        """Remove expired entries."""
        current_time = time.time()
        expired_keys = []
        
        for key, (_, timestamp) in self.cache.items():
            if current_time - timestamp > self.ttl_seconds:
                expired_keys.append(key)
        
        for key in expired_keys:
            del self.cache[key]
        
        if expired_keys:
            logger.debug(f"Cleaned up {len(expired_keys)} expired cache entries")