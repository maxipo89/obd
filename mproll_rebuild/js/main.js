document.addEventListener('DOMContentLoaded', () => {
    const header = document.querySelector('.site-header');
    const mobileBtn = document.querySelector('.mobile-menu-btn');
    const navLinks = document.querySelector('.nav-links');
    const navCta = document.querySelector('.nav-cta');
    const links = document.querySelectorAll('.nav-links a');

    // Mobile Menu Toggle
    if (mobileBtn) {
        mobileBtn.addEventListener('click', () => {
            mobileBtn.classList.toggle('active');
            navLinks.classList.toggle('active');
            if (navCta) navCta.classList.toggle('active');
        });
    }

    // Close menu on link click
    links.forEach(link => {
        link.addEventListener('click', () => {
            mobileBtn.classList.remove('active');
            navLinks.classList.remove('active');
            if (navCta) navCta.classList.remove('active');
        });
    });

    // Shrink header on scroll
    window.addEventListener('scroll', () => {
        if (window.scrollY > 50) {
            header.style.boxShadow = '0 10px 30px rgba(0,0,0,0.08)';
            header.style.background = 'rgba(255, 255, 255, 0.98)';
            header.style.height = '70px';
        } else {
            header.style.boxShadow = 'none';
            header.style.background = 'rgba(255, 255, 255, 0.9)';
            header.style.height = '80px';
        }
    });

    // Robust Scroll to anchor (using scrollIntoView to respect scroll-margin-top)
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function (e) {
            const targetId = this.getAttribute('href');
            if (targetId === '#') return;
            
            const targetEl = document.querySelector(targetId);
            if (targetEl) {
                e.preventDefault();
                // We add a tiny delay to allow the mobile menu to close and layout to stabilize
                setTimeout(() => {
                    targetEl.scrollIntoView({ behavior: 'smooth' });
                    // Self-correction for layout shifts during scroll
                    setTimeout(() => {
                        targetEl.scrollIntoView({ behavior: 'smooth' });
                    }, 600);
                }, 300);
            }
        });
    });

    // Fix for subpage landing (re-calculate scroll if loading from hash)
    if (window.location.hash) {
        setTimeout(() => {
            const hashTarget = document.querySelector(window.location.hash);
            if (hashTarget) {
                hashTarget.scrollIntoView({ behavior: 'smooth' });
            }
        }, 500); // 500ms allows most layout shifts to happen
    }

    // Intersection Observer for scroll animations
    const fadeElements = document.querySelectorAll('.fade-in');
    const appearOptions = { threshold: 0, rootMargin: '0px 0px -100px 0px' };
    const appearOnScroll = new IntersectionObserver(function(entries, observer) {
        entries.forEach(entry => {
            if (!entry.isIntersecting) return;
            entry.target.classList.add('appear');
            observer.unobserve(entry.target);
        });
    }, appearOptions);
    fadeElements.forEach(el => appearOnScroll.observe(el));
});