use poem::{Response, handler, web::Html};

#[handler]
pub fn index_handler() -> Html<&'static str> {
    Html(include_str!("../ui/index.html"))
}

#[handler]
pub fn app_js_handler() -> Response {
    Response::builder()
        .content_type("application/javascript; charset=utf-8")
        .body(include_str!("../ui/app.js"))
}

#[handler]
pub fn styles_css_handler() -> Response {
    Response::builder()
        .content_type("text/css; charset=utf-8")
        .body(include_str!("../ui/styles.css"))
}
