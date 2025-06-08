from flask import Flask, request, jsonify

app = Flask(__name__)


@app.route("/", methods=["POST"])
def receive_order():
    # JSON 데이터가 맞는지 확인
    if not request.is_json:
        return (
            jsonify(
                {"status": "fail", "message": "Content-Type must be application/json"}
            ),
            400,
        )

    # 주문 정보 파싱
    order_data = request.get_json()
    print("수신된 주문 정보:", order_data)

    # (여기서 DB 저장, 영수증 발행 등 추가 처리 가능)

    # 응답 반환
    return (
        jsonify({"status": "success", "message": "주문이 정상적으로 접수되었습니다."}),
        200,
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5003, debug=True)
